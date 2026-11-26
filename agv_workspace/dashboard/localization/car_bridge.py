import hmac
import hashlib
import json
import math
import os
from pathlib import Path
import signal
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer


def yaw(rotation):
    return math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                      1 - 2 * (rotation.y ** 2 + rotation.z ** 2))


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def require_current_stamp(stamp, now_ns):
    if not 0 <= (now_ns - stamp_ns(stamp)) / 1e9 <= 0.5:
        raise ValueError("source timestamp is stale or in the future")


def amcl_payload(verification):
    posterior = verification['posterior']
    if posterior['frame_id'] != "map":
        raise ValueError("AMCL pose is not in map frame")
    if posterior['stamp_ns'] != verification['observation_stamp_ns']:
        raise ValueError('AMCL pose and verification timestamps differ')
    covariance = list(posterior['covariance'])
    values = list(posterior['pose'])
    if len(values) != 3 or len(covariance) != 36:
        raise ValueError('invalid AMCL pose or covariance dimensions')
    if not all(math.isfinite(value) for value in values + covariance):
        raise ValueError("nonfinite AMCL pose or covariance")
    if any(covariance[index] < 0 for index in (0, 7, 35)):
        raise ValueError("negative AMCL covariance")
    return dict(zip(("x", "y", "yaw"), values), stamp_ns=posterior['stamp_ns'],
                frame_id=posterior['frame_id'], covariance=covariance,
                observation_age_s=verification['observation_age_s'],
                yaw_std_deg=math.degrees(math.sqrt(covariance[35])))


class ScanPairing:
    def __init__(self):
        self.pending = None
        self.skipped = 0

    def offer(self, message, received):
        if self.pending is None:
            self.pending = (received, message)
        else:
            self.skipped += 1

    def take(self, now, now_ns, available):
        if self.pending is None:
            return None
        received, message = self.pending
        try:
            if now - received > 0.5:
                raise ValueError('scan expired before its transform arrived')
            require_current_stamp(message.header.stamp, now_ns)
        except ValueError:
            self.pending = None
            raise
        if not available(message):
            return None
        self.pending = None
        return received, message


class State:
    def __init__(self):
        self.lock = threading.Lock()
        self.values = {}
        self.errors = {}
        self.map_changed_ns = 0

    def set_map(self, value, now_ns):
        with self.lock:
            previous = self.values.get('map')
            changed = previous is not None and previous[1]['digest'] != value['digest']
            if changed:
                self.map_changed_ns = now_ns
                for name in ('amcl', 'scan', 'robot_tf', 'verification', 'scan_pairing'):
                    self.values.pop(name, None)
                    self.errors.pop(name, None)
            self.values['map'] = (time.monotonic(), value)
            self.errors.pop('map', None)
            return changed

    def put(self, name, value, received=None):
        with self.lock:
            self.values[name] = (time.monotonic() if received is None else received, value)
            self.errors.pop(name, None)

    def fail(self, name, error):
        with self.lock:
            self.errors[name] = str(error)
            self.values.pop(name, None)

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            result = {"mode": "localization", "motion_api": False, "errors": dict(self.errors)}
            for name, (received, value) in self.values.items():
                age = now - received
                if name in ("scan", "scan_pairing", "robot_tf", "chassis", "verification") and age > 0.5:
                    result["errors"][name] = "data expired"
                    continue
                result[name] = dict(value, receive_age_s=age)
            if ('verification' in result and result['verification']['map_digest'] !=
                    result.get('map', {}).get('digest')):
                result.pop('verification')
                result['errors']['verification'] = 'verification belongs to a different map'
            verification = result.get('verification', {})
            if (verification.get('posterior_source') and verification.get('posterior') is not None and
                    verification.get('phase') not in ('error', 'input_stale', 'waiting_map')):
                try:
                    posterior = amcl_payload(verification)
                    if posterior['stamp_ns'] >= self.map_changed_ns:
                        result['amcl'] = posterior
                except (KeyError, TypeError, ValueError) as error:
                    result['errors']['amcl'] = str(error)
            result["data_ready"] = all(name in result for name in ("map", "scan", "robot_tf", "amcl"))
            result['tracking_valid'] = (result['data_ready'] and
                                        verification.get('phase') == 'verified_pair' and
                                        verification.get('pair_verified') is True)
            result['motion_authorized'] = False
            result["localization_accepted"] = False
            return result


class Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if not hmac.compare_digest(self.headers.get("X-Test-Token", ""), self.server.token):
            self.send_error(403)
            return
        if self.path != "/api/state":
            self.send_error(404)
            return
        body = json.dumps(self.server.state.snapshot(), allow_nan=False).encode()
        self.send_response(200)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, *_):
        pass


def main():
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import OccupancyGrid
    from sensor_msgs.msg import LaserScan
    from std_msgs.msg import String
    from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
    from tf2_ros import Buffer, TransformException, TransformListener

    rclpy.init()
    node = rclpy.create_node("localization_web_observer")
    state = State()
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    pairing = ScanPairing()

    def occupancy(message):
        origin = message.info.origin
        if (message.header.frame_id != "map" or message.info.resolution <= 0 or
                len(message.data) != message.info.width * message.info.height):
            raise ValueError("invalid occupancy grid")
        changed = state.set_map({"width": message.info.width, "height": message.info.height,
                          "resolution": message.info.resolution, "data": list(message.data),
                          "digest": hashlib.sha256(json.dumps(dict(
                              width=message.info.width, height=message.info.height,
                              resolution=message.info.resolution, data=list(message.data),
                              origin=[origin.position.x, origin.position.y, yaw(origin.orientation)]),
                              sort_keys=True, separators=(',', ':')).encode()).hexdigest(),
                          "origin": {"x": origin.position.x, "y": origin.position.y,
                                     "yaw": yaw(origin.orientation)}}, node.get_clock().now().nanoseconds)
        if changed:
            pairing.pending = None

    def scan(message):
        if stamp_ns(message.header.stamp) < state.map_changed_ns:
            return
        pairing.offer(message, time.monotonic())

    def diagnostics(message):
        for entry in message.status:
            if entry.name == "feedbot_chassis/MCU":
                state.put("chassis", {"message": entry.message,
                                      "values": {value.key: value.value for value in entry.values}})

    def verification(message):
        try:
            result = json.loads(message.data)
            for key in ('phase', 'reason', 'map_digest', 'posterior_source',
                        'posterior_generation', 'observation_stamp_ns', 'pair_verified',
                        'posterior', 'observation_age_s'):
                if key not in result:
                    raise ValueError('incomplete verification status')
            state.put('verification', result)
        except (ValueError, TypeError) as error:
            state.fail('verification', error)

    def update():
        try:
            transform = buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            require_current_stamp(transform.header.stamp, node.get_clock().now().nanoseconds)
            if stamp_ns(transform.header.stamp) < state.map_changed_ns:
                raise ValueError('TF predates current map')
            state.put("robot_tf", {"x": transform.transform.translation.x,
                                    "y": transform.transform.translation.y,
                                    "yaw": yaw(transform.transform.rotation),
                                    "stamp_ns": stamp_ns(transform.header.stamp)})
        except (TransformException, ValueError) as error:
            state.fail("robot_tf", error)
        try:
            selected = pairing.take(time.monotonic(), node.get_clock().now().nanoseconds,
                                    lambda message: buffer.can_transform(
                                        "map", message.header.frame_id,
                                        rclpy.time.Time.from_msg(message.header.stamp)))
            state.put('scan_pairing', dict(waiting_for_tf=pairing.pending is not None,
                                          skipped_display_scans=pairing.skipped))
            if selected is None:
                return
            received, message = selected
            transform = buffer.lookup_transform("map", message.header.frame_id,
                                                rclpy.time.Time.from_msg(message.header.stamp)).transform
        except (ValueError, TransformException) as error:
            state.fail("scan", error)
            return
        angle = yaw(transform.rotation)
        points = []
        for index, distance in enumerate(message.ranges):
            if math.isfinite(distance) and message.range_min <= distance <= message.range_max:
                beam = message.angle_min + index * message.angle_increment + angle
                points.append([transform.translation.x + distance * math.cos(beam),
                               transform.translation.y + distance * math.sin(beam)])
        state.put("scan", {"stamp_ns": stamp_ns(message.header.stamp), "points": points}, received)

    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [node.create_subscription(OccupancyGrid, "/map", occupancy, retained),
                     node.create_subscription(LaserScan, "/scan", scan, qos_profile_sensor_data),
                     node.create_subscription(String, "/localization/verification", verification, 1),
                     node.create_subscription(DiagnosticArray, "/diagnostics", diagnostics, 10)]
    timer = node.create_timer(0.05, update)
    server = ThreadingHTTPServer(("10.7.5.88", 8098), Handler)
    server.token = Path(os.environ["FEEDBOT_TEST_TOKEN_FILE"]).read_text().strip()
    server.state = state
    worker = threading.Thread(target=server.serve_forever, daemon=True)
    worker.start()
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=0.05)
    finally:
        server.shutdown()
        server.server_close()
        worker.join()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
