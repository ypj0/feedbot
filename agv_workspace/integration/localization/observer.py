import concurrent.futures
import json
import math
import signal
import threading

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from verification import Verification, evaluate


def stamp_ns(stamp):
    return stamp.sec * 1000000000 + stamp.nanosec


def yaw(rotation):
    return math.atan2(2 * (rotation.w * rotation.z + rotation.x * rotation.y),
                      1 - 2 * (rotation.y ** 2 + rotation.z ** 2))


def main():
    rclpy.init()
    node = rclpy.create_node('localization_verifier')
    state = Verification()
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    publisher = node.create_publisher(String, '/localization/verification', 1)
    pool = concurrent.futures.ThreadPoolExecutor(max_workers=1)
    work = {'future': None, 'job': None}

    def failure(source, error):
        state.errors[source] = str(error)
        node.get_logger().error(str(error))

    def sync_posterior_source():
        endpoints = node.get_publishers_info_by_topic('/amcl_pose')
        source = bytes(endpoints[0].endpoint_gid).hex() if len(endpoints) == 1 else None
        state.set_posterior_source(source, node.get_clock().now().nanoseconds)

    def map_received(message):
        if (message.header.frame_id != 'map' or message.info.resolution <= 0 or
                len(message.data) != message.info.width * message.info.height):
            failure('map', 'invalid map geometry or frame')
            return
        origin = message.info.origin
        state.set_map(dict(width=message.info.width, height=message.info.height,
                           resolution=message.info.resolution,
                           origin=[origin.position.x, origin.position.y, yaw(origin.orientation)],
                           data=list(message.data)), node.get_clock().now().nanoseconds)

    def posterior_received(message):
        sync_posterior_source()
        if state.posterior_source is None:
            return
        pose = message.pose.pose
        try:
            state.posterior(stamp_ns(message.header.stamp), message.header.frame_id,
                            [pose.position.x, pose.position.y, yaw(pose.orientation)],
                            message.pose.covariance)
        except ValueError as error:
            failure('posterior', error)

    def scan_received(message):
        source_ns = stamp_ns(message.header.stamp)
        if not 0 <= node.get_clock().now().nanoseconds - source_ns < 500000000:
            failure('scan', 'scan source timestamp is stale or in the future')
            return
        try:
            transform = buffer.lookup_transform(
                'base_link', message.header.frame_id,
                rclpy.time.Time.from_msg(message.header.stamp)).transform
        except TransformException as error:
            failure('scan', error)
            return
        rotation = transform.rotation
        angle = yaw(rotation)
        if any(abs(value) > 1e-6 for value in (
                transform.translation.x - .27, transform.translation.y,
                rotation.x, rotation.y, angle)):
            failure('scan', 'legacy geometry requires laser x=0.27, y=0, roll=pitch=yaw=0')
            return
        points = []
        for index, distance in enumerate(message.ranges):
            if math.isfinite(distance) and message.range_min <= distance <= message.range_max:
                beam = message.angle_min + index * message.angle_increment + angle
                points.append([transform.translation.x + distance * math.cos(beam),
                               transform.translation.y + distance * math.sin(beam)])
        state.scan(source_ns, points)

    def tick():
        sync_posterior_source()
        future = work['future']
        if future is not None and future.done():
            try:
                result = future.result()
                if state.complete(work['job'], result):
                    print(json.dumps(dict(map_digest=work['job']['map_digest'],
                                          stamp_ns=work['job']['stamp_ns'], result=result)), flush=True)
            except Exception as error:
                if (work['job']['generation'] == state.generation and
                        work['job']['posterior_generation'] == state.posterior_generation):
                    failure('evaluation', error)
            work['future'] = None
        if work['future'] is None:
            job = state.prepare(node.get_clock().now().nanoseconds)
            if job is not None:
                work['job'] = job
                work['future'] = pool.submit(evaluate, job)
        publisher.publish(String(data=json.dumps(state.snapshot(node.get_clock().now().nanoseconds),
                                                 ensure_ascii=False, allow_nan=False)))

    retained = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
    subscriptions = [node.create_subscription(OccupancyGrid, '/map', map_received, retained),
                     node.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', posterior_received, retained),
                     node.create_subscription(LaserScan, '/scan', scan_received, qos_profile_sensor_data)]
    timer = node.create_timer(.2, tick)
    stop = threading.Event()
    signal.signal(signal.SIGINT, lambda *_: stop.set())
    signal.signal(signal.SIGTERM, lambda *_: stop.set())
    try:
        while rclpy.ok() and not stop.is_set():
            rclpy.spin_once(node, timeout_sec=.05)
    finally:
        pool.shutdown(wait=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
