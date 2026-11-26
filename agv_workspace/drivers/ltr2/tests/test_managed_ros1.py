#!/usr/bin/env python
"""Loopback-only ROS1 integration fixture for ltr2_managed_ros1.

Usage: test_managed_ros1.py /path/to/ltr2_managed_ros1 /path/to/raw.bin
"""
from __future__ import print_function

import binascii
import fcntl
import json
import os
import signal
import shutil
import socket
import struct
import subprocess
import sys
import tempfile
import threading
import time

try:
    import Queue as queue
except ImportError:
    import queue


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def wait_until(predicate, timeout, message, interval=0.01):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if predicate():
            return
        time.sleep(interval)
    raise AssertionError(message)


def unused_port(sock_type):
    probe = socket.socket(socket.AF_INET, sock_type)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def array_bytes(value):
    if sys.version_info[0] < 3:
        return str(value)
    return bytes(value)


def captured_blocks(path):
    with open(path, "rb") as source:
        raw = source.read()
    position = 0
    current_frame = None
    blocks = []
    while position < len(raw):
        require(position + 12 <= len(raw), "truncated raw.bin record header")
        _, packet_length = struct.unpack_from("<dI", raw, position)
        position += 12
        require(position + packet_length <= len(raw), "truncated raw.bin packet")
        packet = raw[position:position + packet_length]
        position += packet_length
        if len(packet) < 42 or packet[12:14] != b"\x08\x00":
            continue
        version_and_length = packet[14]
        if not isinstance(version_and_length, int):
            version_and_length = ord(version_and_length)
        header_length = (version_and_length & 15) * 4
        if header_length < 20 or len(packet) < 14 + header_length + 8 or packet[23:24] != b"\x11":
            continue
        udp_offset = 14 + header_length
        udp_length = struct.unpack_from("!H", packet, udp_offset + 4)[0]
        payload = packet[udp_offset + 8:udp_offset + udp_length]
        if len(payload) != 1040 or payload[:2] != b"\xff\xff":
            continue
        frame = struct.unpack_from("<H", payload, 2)[0]
        index = payload[4]
        if not isinstance(index, int):
            index = ord(index)
        if index == 0:
            current_frame = frame
            blocks = [payload]
        elif current_frame == frame and index == len(blocks):
            blocks.append(payload)
        else:
            current_frame = None
            blocks = []
        if len(blocks) == 8:
            return blocks
    raise AssertionError("raw.bin contains no complete eight-block LT-R2-B frame")


def rewritten_block(template, frame, index, device_stamp):
    packet = bytearray(template)
    struct.pack_into("<H", packet, 2, frame)
    packet[4] = index
    struct.pack_into("<I", packet, 8, device_stamp)
    struct.pack_into("<H", packet, 12, 0)
    checksum = binascii.crc_hqx(array_bytes(packet), 0xffff)
    struct.pack_into("<H", packet, 12, checksum)
    return array_bytes(packet)


class MockLidar(object):
    def __init__(self, templates, data_port):
        self.templates = templates
        self.data_port = data_port
        self.lock = threading.Lock()
        self.stop_event = threading.Event()
        self.calls = []
        self.errors = []
        self.contacts = 0
        self.connections = []
        self.expected_disconnects = set()
        self.measurement = True
        self.streaming = True
        self.suppress = True

        self.control = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.control.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.control.bind(("127.0.0.1", 0))
        self.control.listen(4)
        self.control.settimeout(0.1)
        self.control_port = self.control.getsockname()[1]

        self.udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp.bind(("127.0.0.1", 0))
        self.source_port = self.udp.getsockname()[1]

        self.control_thread = threading.Thread(target=self._accept_loop)
        self.control_thread.daemon = True
        self.sender_thread = threading.Thread(target=self._send_loop)
        self.sender_thread.daemon = True
        self.control_thread.start()
        self.sender_thread.start()

    def snapshot(self):
        with self.lock:
            return {
                "calls": list(self.calls),
                "errors": list(self.errors),
                "contacts": self.contacts,
                "connections": len(self.connections),
                "measurement": self.measurement,
                "streaming": self.streaming,
                "suppress": self.suppress,
            }

    def set_suppress(self, value):
        with self.lock:
            self.suppress = value

    def close_control_connections(self):
        with self.lock:
            connections = list(self.connections)
            self.expected_disconnects.update(id(connection) for connection in connections)
        require(connections, "no control TCP connection to disconnect")
        for connection in connections:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except socket.error:
                pass
            try:
                connection.close()
            except socket.error:
                pass
        return len(connections)

    def _accept_loop(self):
        while not self.stop_event.is_set():
            try:
                connection, _ = self.control.accept()
            except socket.timeout:
                continue
            except socket.error:
                if self.stop_event.is_set():
                    return
                raise
            with self.lock:
                self.contacts += 1
                self.connections.append(connection)
            worker = threading.Thread(target=self._serve, args=(connection,))
            worker.daemon = True
            worker.start()

    def _serve(self, connection):
        stream = None
        try:
            stream = connection.makefile("rb")
            while not self.stop_event.is_set():
                line = stream.readline(65537)
                if not line:
                    break
                require(len(line) <= 65536, "LDCP request line exceeds fixture bound")
                delimiter = b"," + b"0" + b":" + b","
                checksum_header, body = line.split(delimiter, 1)
                length_text, payload_tail = body.split(b":", 1)
                length = int(length_text)
                require(len(payload_tail) == length + 3 and payload_tail[length:] == b",\r\n",
                        "invalid LDCP netstring")
                payload = payload_tail[:length]
                require(checksum_header.startswith(b"15:checksum=0x"), "missing LDCP checksum")
                expected = int(checksum_header.split(b"0x", 1)[1], 16)
                require(expected == binascii.crc_hqx(payload, 0xffff), "bad LDCP checksum")
                request = json.loads(payload.decode("utf-8"))
                method = request["method"]
                entry = request.get("params", {}).get("entry")
                with self.lock:
                    self.calls.append((method, entry))
                    values = {
                        "identity.model": "LT-R2-B",
                        "transport.oob.enabled": True,
                        "transport.oob.targetAddress": "127.0.0.1",
                        "transport.oob.targetPort": self.data_port,
                    }
                    if method in ("device/queryInfo", "settings/get"):
                        result = values[entry]
                    elif method == "scan/startMeasurement":
                        self.measurement = True
                        result = True
                    elif method == "scan/startStreaming":
                        self.streaming = True
                        result = True
                    elif method == "scan/stopStreaming":
                        self.streaming = False
                        result = True
                    elif method == "scan/stopMeasurement":
                        self.measurement = False
                        result = True
                    else:
                        raise AssertionError("unexpected LDCP method " + method)
                response = json.dumps({"jsonrpc": "2.0", "id": request["id"], "result": result},
                                      separators=(",", ":")).encode("utf-8")
                header = ("15:checksum=0x%04X,0:,%d:" %
                          (binascii.crc_hqx(response, 0xffff), len(response))).encode("ascii")
                connection.sendall(header + response + b",\r\n")
        except Exception as error:
            if not self.stop_event.is_set():
                with self.lock:
                    expected_socket_error = (id(connection) in self.expected_disconnects and
                                             isinstance(error, socket.error))
                    if not expected_socket_error:
                        self.errors.append(repr(error))
        finally:
            if stream is not None:
                try:
                    stream.close()
                except Exception:
                    pass
            try:
                connection.close()
            except Exception:
                pass
            with self.lock:
                if connection in self.connections:
                    self.connections.remove(connection)
                self.expected_disconnects.discard(id(connection))

    def _send_loop(self):
        frame = 1000
        device_stamp = 1000000
        index = 0
        while not self.stop_event.is_set():
            with self.lock:
                enabled = self.measurement and self.streaming and not self.suppress
            if not enabled:
                if index:
                    frame = (frame + 1) & 0xffff
                    device_stamp = (device_stamp + 100000) & 0xffffffff
                    index = 0
                self.stop_event.wait(0.002)
                continue
            try:
                packet = rewritten_block(
                    self.templates[index], frame, index,
                    (device_stamp + index * 12500) & 0xffffffff)
                self.udp.sendto(packet, ("127.0.0.1", self.data_port))
            except socket.error as error:
                if not self.stop_event.is_set():
                    with self.lock:
                        self.errors.append(repr(error))
                return
            index += 1
            if index == 8:
                frame = (frame + 1) & 0xffff
                device_stamp = (device_stamp + 100000) & 0xffffffff
                index = 0
                self.stop_event.wait(0.02)
            else:
                self.stop_event.wait(0.008)

    def close(self):
        self.stop_event.set()
        try:
            self.control.close()
        except Exception:
            pass
        with self.lock:
            connections = list(self.connections)
        for connection in connections:
            try:
                connection.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                connection.close()
            except Exception:
                pass
        try:
            self.udp.close()
        except Exception:
            pass
        self.control_thread.join(1.0)
        self.sender_thread.join(1.0)


class RosMessages(object):
    def __init__(self):
        self.lock = threading.Lock()
        self.scans = []
        self.diagnostics = []

    def scan(self, message):
        with self.lock:
            self.scans.append(message)

    def diagnostic(self, message):
        with self.lock:
            self.diagnostics.append(message)

    def scan_count(self):
        with self.lock:
            return len(self.scans)

    def latest_scan(self):
        with self.lock:
            return self.scans[-1] if self.scans else None

    def diagnostic_values(self):
        with self.lock:
            if not self.diagnostics or not self.diagnostics[-1].status:
                return {}
            status = self.diagnostics[-1].status[0]
            values = dict((item.key, item.value) for item in status.values)
            values["state"] = status.message
            return values


def wait_master(uri, process, timeout):
    if sys.version_info[0] < 3:
        import xmlrpclib as xmlrpc_client
    else:
        import xmlrpc.client as xmlrpc_client
    proxy = xmlrpc_client.ServerProxy(uri)
    deadline = time.time() + timeout
    while time.time() < deadline:
        require(process.poll() is None, "private roscore exited during startup")
        try:
            code, _, _ = proxy.getPid("/ltr2_managed_fixture")
            if code == 1:
                return
        except Exception:
            pass
        time.sleep(0.05)
    raise AssertionError("private roscore did not become ready")


def stop_process(process, process_group=False, timeout=3.0):
    if process is None:
        return
    try:
        if process_group:
            os.killpg(process.pid, signal.SIGTERM)
        elif process.poll() is None:
            process.terminate()
    except OSError:
        pass
    deadline = time.time() + timeout
    while process.poll() is None and time.time() < deadline:
        time.sleep(0.05)
    try:
        if process_group:
            os.killpg(process.pid, signal.SIGKILL)
        elif process.poll() is None:
            process.kill()
    except OSError:
        pass
    if process.poll() is None:
        process.wait()


def read_status(path):
    try:
        with open(path, "r") as source:
            return json.load(source)
    except (IOError, ValueError):
        return None


def service_result(rospy, trigger_type, name):
    return rospy.ServiceProxy(name, trigger_type)()


def async_service(rospy, trigger_type, name):
    results = queue.Queue()

    def invoke():
        try:
            results.put((service_result(rospy, trigger_type, name), None))
        except Exception as error:
            results.put((None, error))

    thread = threading.Thread(target=invoke)
    thread.daemon = True
    thread.start()
    return thread, results


def run_fixture(executable, raw_path):
    require(os.path.isfile(executable) and os.access(executable, os.X_OK),
            "managed ROS1 executable is missing or not executable")
    require(os.path.isfile(raw_path), "recorded raw.bin is missing")
    templates = captured_blocks(raw_path)
    root = tempfile.mkdtemp(prefix="ltr2-managed-ros1-")
    roscore = None
    driver = None
    mock = None
    master_log = None
    driver_log = None
    rospy = None
    try:
        owner_directory = os.path.join(root, "owner")
        ros_home = os.path.join(root, "ros-home")
        ros_log = os.path.join(root, "ros-log")
        os.mkdir(owner_directory)
        os.mkdir(ros_home)
        os.mkdir(ros_log)
        token = binascii.hexlify(os.urandom(24)).decode("ascii")
        require(len(token) == 48 and token == token.lower(), "invalid fixture owner token")

        data_port = unused_port(socket.SOCK_DGRAM)
        mock = MockLidar(templates, data_port)
        master_port = unused_port(socket.SOCK_STREAM)
        master_uri = "http://127.0.0.1:%d/" % master_port
        environment = os.environ.copy()
        environment["ROS_MASTER_URI"] = master_uri
        environment["ROS_IP"] = "127.0.0.1"
        environment["ROS_HOME"] = ros_home
        environment["ROS_LOG_DIR"] = ros_log
        environment.pop("ROS_HOSTNAME", None)
        environment.pop("ROS_NAMESPACE", None)
        os.environ.pop("ROS_HOSTNAME", None)
        os.environ.pop("ROS_NAMESPACE", None)
        os.environ.update(environment)

        master_log = open(os.path.join(root, "roscore.log"), "w+")
        roscore = subprocess.Popen(["roscore", "-p", str(master_port)], env=environment,
                                   stdout=master_log, stderr=subprocess.STDOUT,
                                   preexec_fn=os.setsid)
        wait_master(master_uri, roscore, 10.0)

        driver_log = open(os.path.join(root, "driver.log"), "w+")
        command = [
            executable,
            "__name:=ltr2_managed_driver",
            "_device_address:=127.0.0.1",
            "_target_address:=127.0.0.1",
            "_control_port:=%d" % mock.control_port,
            "_data_port:=%d" % data_port,
            "_source_port:=%d" % mock.source_port,
            "_bind_address:=127.0.0.1",
            "_owner_directory:=" + owner_directory,
            "_owner_token:=" + token,
            "_startup_timeout:=1.0",
        ]
        driver = subprocess.Popen(command, env=environment, stdout=driver_log,
                                  stderr=subprocess.STDOUT, preexec_fn=os.setsid)

        import rospy as imported_rospy
        from diagnostic_msgs.msg import DiagnosticArray
        from sensor_msgs.msg import LaserScan
        from std_srvs.srv import Trigger
        rospy = imported_rospy
        rospy.init_node("ltr2_managed_fixture", argv=[sys.argv[0]], anonymous=True,
                        disable_signals=True)
        rospy.set_param("/use_sim_time", False)
        services = ["/ltr2_driver/configure", "/ltr2_driver/activate",
                    "/ltr2_driver/deactivate", "/ltr2_driver/cleanup"]
        for name in services:
            rospy.wait_for_service(name, timeout=10.0)
        messages = RosMessages()
        scan_subscription = rospy.Subscriber("/candidate/ltr2/managed_scan", LaserScan,
                                             messages.scan, queue_size=20)
        diagnostic_subscription = rospy.Subscriber(
            "/candidate/ltr2/managed_diagnostics", DiagnosticArray,
            messages.diagnostic, queue_size=20)
        require(scan_subscription.resolved_name == "/candidate/ltr2/managed_scan",
                "managed scan topic was remapped")
        require(diagnostic_subscription.resolved_name == "/candidate/ltr2/managed_diagnostics",
                "managed diagnostic topic was remapped")

        status_path = os.path.join(owner_directory, "status.json")
        sentinel = "fixture-must-not-be-overwritten\n"
        with open(status_path, "w") as output:
            output.write(sentinel)
        result = service_result(rospy, Trigger, "/ltr2_driver/configure")
        require(not result.success and "permit" in result.message, "missing permit was accepted")
        with open(status_path, "r") as source:
            require(source.read() == sentinel, "status was written without owning hardware.lock")
        require(mock.snapshot()["contacts"] == 0, "missing permit contacted control TCP")

        permit_path = os.path.join(owner_directory, "permit")
        with open(permit_path, "w") as output:
            output.write(token + "\n")
        lock_path = os.path.join(owner_directory, "hardware.lock")
        held_lock = open(lock_path, "a+")
        fcntl.flock(held_lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        try:
            result = service_result(rospy, Trigger, "/ltr2_driver/configure")
            require(not result.success and "lock" in result.message, "hardware lock contention was accepted")
            with open(status_path, "r") as source:
                require(source.read() == sentinel, "lock loser overwrote supervisor status")
            require(mock.snapshot()["contacts"] == 0, "lock loser contacted control TCP")
        finally:
            fcntl.flock(held_lock, fcntl.LOCK_UN)
            held_lock.close()

        busy_udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        busy_udp.bind(("127.0.0.1", data_port))
        try:
            result = service_result(rospy, Trigger, "/ltr2_driver/configure")
            require(not result.success and "UDP" in result.message, "UDP port contention was accepted")
            require(mock.snapshot()["contacts"] == 0, "UDP bind loser contacted control TCP")
        finally:
            busy_udp.close()

        result = service_result(rospy, Trigger, "/ltr2_driver/configure")
        require(result.success, "valid configure failed: " + result.message)
        expected_validation = [
            ("device/queryInfo", "identity.model"),
            ("settings/get", "transport.oob.enabled"),
            ("settings/get", "transport.oob.targetAddress"),
            ("settings/get", "transport.oob.targetPort"),
        ]
        expected_stop = [
            ("scan/stopStreaming", None),
            ("scan/stopMeasurement", None),
        ]
        expected_configure = expected_validation + expected_stop
        wait_until(lambda: len(mock.snapshot()["calls"]) >= len(expected_configure), 2.0,
                   "configure did not validate and stop the device")
        require(mock.snapshot()["calls"][:len(expected_configure)] == expected_configure,
                "unexpected configure RPCs: %r" %
                (mock.snapshot()["calls"][:len(expected_configure)],))
        require(not mock.snapshot()["measurement"] and not mock.snapshot()["streaming"],
                "configure did not physically stop measurement and streaming")

        cleanup = service_result(rospy, Trigger, "/ltr2_driver/cleanup")
        require(cleanup.success and "resources released" in cleanup.message,
                "inactive cleanup failed after the confirmed hardware stop")
        wait_until(lambda: mock.snapshot()["connections"] == 0, 1.0,
                   "inactive cleanup did not close control TCP")
        require(mock.snapshot()["calls"] == expected_configure and
                not mock.snapshot()["measurement"] and not mock.snapshot()["streaming"],
                "inactive cleanup issued writes or lost the confirmed stop")
        wait_until(lambda: read_status(status_path) is not None and
                   read_status(status_path).get("state") == "disconnected", 1.0,
                   "inactive cleanup did not publish disconnected status")

        second_configure_start = len(mock.snapshot()["calls"])
        result = service_result(rospy, Trigger, "/ltr2_driver/configure")
        require(result.success, "configure after inactive cleanup failed: " + result.message)
        wait_until(lambda: len(mock.snapshot()["calls"]) >=
                   second_configure_start + len(expected_configure), 2.0,
                   "configure after inactive cleanup did not validate and stop the device")
        require(mock.snapshot()["calls"][second_configure_start:] == expected_configure,
                "unexpected reconfigure RPCs: %r" %
                (mock.snapshot()["calls"][second_configure_start:],))
        wait_until(lambda: read_status(status_path) is not None and
                   read_status(status_path).get("state") == "inactive", 2.0,
                   "reconfigure did not publish inactive status")
        configured_status = read_status(status_path)
        require(configured_status["ready"] is False and configured_status["healthy"] is True and
                configured_status["pid"] == driver.pid and configured_status["scans"] == 0,
                "configured status contract is wrong")

        activation_thread, activation_results = async_service(
            rospy, Trigger, "/ltr2_driver/activate")
        wait_until(lambda: mock.snapshot()["measurement"] and mock.snapshot()["streaming"],
                   2.0, "activate did not start measurement and streaming")
        time.sleep(0.2)
        require(activation_thread.is_alive(), "activate succeeded without a new complete scan")
        mock.set_suppress(False)
        activation_thread.join(2.0)
        require(not activation_thread.is_alive(), "activate exceeded startup timeout")
        activation, activation_error = activation_results.get_nowait()
        require(activation_error is None and activation.success,
                "fresh-data activation failed: %r" % (activation_error or activation.message,))
        wait_until(lambda: messages.scan_count() >= 1, 2.0, "active driver published no scan")
        scan = messages.latest_scan()
        increment = 6.283185307179586 / 2048.0
        require(len(scan.ranges) == 1075 and len(scan.intensities) == 1075,
                "managed scan does not contain 1075 range/intensity samples")
        require(abs(scan.angle_increment - increment) < 1.0e-6 and
                abs(scan.angle_min - (-537.0 * increment)) < 1.0e-5 and
                abs(scan.angle_max - (537.0 * increment)) < 1.0e-5 and
                abs(scan.angle_max - (scan.angle_min + 1074.0 * scan.angle_increment)) < 1.0e-5,
                "managed scan geometry is inconsistent")
        require(scan.header.frame_id == "scan" and abs(scan.range_min - 0.05) < 1.0e-6 and
                abs(scan.range_max - 40.0) < 1.0e-6 and 0.025 <= scan.scan_time <= 0.150,
                "managed scan metadata is inconsistent")
        wait_until(lambda: messages.diagnostic_values().get("ready") == "true", 2.0,
                   "active diagnostic never became ready")
        active_diagnostic = messages.diagnostic_values()
        require(active_diagnostic.get("healthy") == "true" and
                active_diagnostic.get("navigation_accepted") == "false",
                "active diagnostic overstates navigation acceptance")

        result = service_result(rospy, Trigger, "/ltr2_driver/deactivate")
        require(result.success, "deactivate failed: " + result.message)
        wait_until(lambda: not mock.snapshot()["measurement"] and
                   not mock.snapshot()["streaming"], 1.0,
                   "deactivate did not stop streaming and measurement")
        time.sleep(0.1)
        inactive_count = messages.scan_count()
        time.sleep(0.4)
        require(messages.scan_count() == inactive_count, "inactive driver replayed a stale scan")
        sleeping_status = read_status(status_path)
        require(sleeping_status["state"] == "sleeping" and sleeping_status["ready"] is False and
                sleeping_status["healthy"] is True, "deactivate did not enter healthy sleep")
        mutation_calls = [item[0] for item in mock.snapshot()["calls"]
                          if item[0].startswith("scan/")]
        require(mutation_calls[-2:] ==
                ["scan/stopStreaming", "scan/stopMeasurement"],
                "deactivate command order is wrong")

        previous_stamp = scan.header.stamp.to_sec()
        result = service_result(rospy, Trigger, "/ltr2_driver/activate")
        require(result.success, "sleep resume failed: " + result.message)
        wait_until(lambda: messages.scan_count() > inactive_count and
                   messages.latest_scan().header.stamp.to_sec() > previous_stamp,
                   2.0, "sleep resume did not publish a new frame")

        mock.set_suppress(True)
        wait_until(lambda: read_status(status_path) is not None and
                   read_status(status_path).get("state") == "fault" and
                   read_status(status_path).get("ready") is False, 2.0,
                   "physical stream loss did not revoke readiness")
        wait_until(lambda: mock.snapshot()["connections"] == 0, 2.0,
                   "stream-loss fault did not close control TCP")
        time.sleep(0.1)
        fault_count = messages.scan_count()
        time.sleep(0.4)
        require(messages.scan_count() == fault_count, "faulted driver published cached output")
        wait_until(lambda: messages.diagnostic_values().get("ready") == "false", 1.0,
                   "fault diagnostic still reports ready")

        cleanup = service_result(rospy, Trigger, "/ltr2_driver/cleanup")
        require("resources released" in cleanup.message, "fault cleanup did not release resources")
        wait_until(lambda: read_status(status_path) is not None and
                   read_status(status_path).get("state") == "disconnected", 1.0,
                   "cleanup did not publish disconnected status")
        mock.set_suppress(False)
        result = service_result(rospy, Trigger, "/ltr2_driver/configure")
        require(result.success, "reconfigure after cleanup failed: " + result.message)
        recovered_before_activation = messages.scan_count()
        time.sleep(0.3)
        require(messages.scan_count() == recovered_before_activation and
                not mock.snapshot()["measurement"] and not mock.snapshot()["streaming"],
                "reconfigure resumed hardware without explicit activation")
        result = service_result(rospy, Trigger, "/ltr2_driver/activate")
        require(result.success, "explicit recovery activation failed: " + result.message)
        wait_until(lambda: messages.scan_count() > recovered_before_activation, 2.0,
                   "recovered driver published no new scan")

        duplicate_baseline = messages.scan_count()
        contacts_before_duplicate = mock.snapshot()["contacts"]
        result = service_result(rospy, Trigger, "/ltr2_driver/activate")
        require(not result.success and "inactive or sleeping" in result.message,
                "duplicate activate was not rejected")
        require(mock.snapshot()["contacts"] == contacts_before_duplicate and
                mock.snapshot()["connections"] == 1 and
                mock.snapshot()["measurement"] and mock.snapshot()["streaming"],
                "duplicate activate disturbed the live control session")
        wait_until(lambda: messages.scan_count() > duplicate_baseline, 1.0,
                   "duplicate activate stopped subsequent scans")
        wait_until(lambda: read_status(status_path) is not None and
                   read_status(status_path).get("state") == "active" and
                   read_status(status_path).get("ready") is True, 1.0,
                   "duplicate activate revoked live readiness")

        tcp_loss_contacts = mock.snapshot()["contacts"]
        require(mock.close_control_connections() == 1,
                "TCP-only loss did not target exactly one control session")
        require(mock.snapshot()["measurement"] and mock.snapshot()["streaming"],
                "TCP-only loss incorrectly stopped the UDP source")
        wait_until(lambda: read_status(status_path) is not None and
                   read_status(status_path).get("state") == "fault" and
                   read_status(status_path).get("ready") is False, 3.0,
                   "TCP-only loss with continuing UDP did not fault")
        require(mock.snapshot()["contacts"] == tcp_loss_contacts,
                "driver automatically reconnected after TCP-only loss")
        time.sleep(0.1)
        tcp_fault_count = messages.scan_count()
        time.sleep(0.4)
        require(messages.scan_count() == tcp_fault_count,
                "control-faulted driver continued publishing UDP scans")

        cleanup = service_result(rospy, Trigger, "/ltr2_driver/cleanup")
        require("resources released" in cleanup.message,
                "TCP-fault cleanup did not release resources")
        result = service_result(rospy, Trigger, "/ltr2_driver/configure")
        require(result.success, "TCP-fault reconfigure failed: " + result.message)
        tcp_recovery_baseline = messages.scan_count()
        time.sleep(0.3)
        require(messages.scan_count() == tcp_recovery_baseline,
                "TCP-fault reconfigure resumed scans without explicit activation")
        result = service_result(rospy, Trigger, "/ltr2_driver/activate")
        require(result.success, "TCP-fault explicit activation failed: " + result.message)
        wait_until(lambda: messages.scan_count() > tcp_recovery_baseline, 2.0,
                   "TCP-fault recovery published no new scan")

        os.unlink(permit_path)
        wait_until(lambda: read_status(status_path) is not None and
                   read_status(status_path).get("state") == "fault" and
                   read_status(status_path).get("ready") is False and
                   read_status(status_path).get("healthy") is False, 2.0,
                   "permit revocation did not enter unhealthy fault")
        wait_until(lambda: mock.snapshot()["connections"] == 0, 2.0,
                   "permit revocation did not close control TCP")
        time.sleep(0.1)
        revoked_count = messages.scan_count()
        time.sleep(0.4)
        require(messages.scan_count() == revoked_count, "revoked driver published cached output")
        service_result(rospy, Trigger, "/ltr2_driver/cleanup")

        snapshot = mock.snapshot()
        require(not snapshot["errors"], "mock lidar errors: %r" % snapshot["errors"])
        require(snapshot["contacts"] == 4, "unexpected control TCP contact count")
        require(driver.poll() is None, "managed driver exited during fixture")
        print(json.dumps({
            "passed": True,
            "control_contacts": snapshot["contacts"],
            "scan_messages": messages.scan_count(),
            "tests": [
                "private ROS master and loopback parameters",
                "missing permit without TCP contact",
                "hardware lock and UDP port contention without TCP contact",
                "identity and OOB validation with confirmed initial stop",
                "inactive cleanup releases without writes or reactivation",
                "activation waits for a new complete scan",
                "1075-point scan geometry",
                "inactive output remains stale-free",
                "sleep and explicit resume",
                "stream-loss readiness revocation",
                "cleanup, reconfigure, and explicit recovery activation",
                "duplicate activation rejection without session disruption",
                "TCP-only loss faults despite continuing UDP",
                "TCP-fault cleanup and explicit reconnect activation",
                "permit revocation fault and control close",
            ],
        }, indent=2, sort_keys=True))
    except Exception:
        if driver_log is not None:
            driver_log.flush()
            driver_log.seek(0)
            print("--- ltr2_managed_ros1 log ---", file=sys.stderr)
            print(driver_log.read()[-6000:], file=sys.stderr)
        if master_log is not None:
            master_log.flush()
            master_log.seek(0)
            print("--- private roscore log ---", file=sys.stderr)
            print(master_log.read()[-3000:], file=sys.stderr)
        raise
    finally:
        if rospy is not None:
            try:
                rospy.signal_shutdown("fixture complete")
            except Exception:
                pass
        stop_process(driver, process_group=True)
        stop_process(roscore, process_group=True)
        if mock is not None:
            mock.close()
        if driver_log is not None:
            driver_log.close()
        if master_log is not None:
            master_log.close()
        shutil.rmtree(root)


def main():
    require(len(sys.argv) == 3,
            "usage: test_managed_ros1.py LTR2_MANAGED_ROS1 RECORDED_RAW_BIN")
    run_fixture(os.path.abspath(sys.argv[1]), os.path.abspath(sys.argv[2]))


if __name__ == "__main__":
    main()
