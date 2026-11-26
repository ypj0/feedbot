#!/usr/bin/env python3
import json
import time

import rclpy
from lifecycle_msgs.srv import ChangeState, GetState


CHANGE_STATE_SERVICE = "/slam_toolbox/change_state"
GET_STATE_SERVICE = "/slam_toolbox/get_state"


def wait_for_result(node, client, request, timeout):
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done():
        raise RuntimeError("slam_toolbox lifecycle request timed out")
    result = future.result()
    if result is None:
        raise RuntimeError("slam_toolbox lifecycle request returned no result")
    return result


def change_state(node, identifier, name):
    client = node.create_client(ChangeState, CHANGE_STATE_SERVICE)
    if not client.wait_for_service(timeout_sec=8):
        raise RuntimeError("slam_toolbox change_state service absent")
    request = ChangeState.Request()
    request.transition.id = identifier
    started = time.monotonic()
    result = wait_for_result(node, client, request, 12)
    if not result.success:
        raise RuntimeError(name + " transition failed")
    return time.monotonic() - started


def require_state(node, expected, name):
    client = node.create_client(GetState, GET_STATE_SERVICE)
    if not client.wait_for_service(timeout_sec=3):
        raise RuntimeError("slam_toolbox get_state service absent")
    result = wait_for_result(node, client, GetState.Request(), 3)
    actual = result.current_state.id
    if actual != expected:
        raise RuntimeError("slam_toolbox state after %s is %s, expected %s" %
                           (name, actual, expected))
    return actual


def activate_mapping(node, report=None):
    if report is None:
        report = {"passed": False, "node": "/slam_toolbox", "transitions": []}
    elapsed = change_state(node, 1, "configure")
    state = require_state(node, 2, "configure")
    report["transitions"].append(
        {"transition": "configure", "elapsed_s": elapsed, "state": state})
    elapsed = change_state(node, 3, "activate")
    state = require_state(node, 3, "activate")
    report["transitions"].append(
        {"transition": "activate", "elapsed_s": elapsed, "state": state})
    report["passed"] = True
    return report


def main():
    rclpy.init()
    node = rclpy.create_node("feedbot_mapping_activation")
    report = {"passed": False, "node": "/slam_toolbox", "transitions": []}
    try:
        activate_mapping(node, report)
        print(json.dumps(report))
    except BaseException as error:
        report["error"] = "%s: %s" % (type(error).__name__, error)
        print(json.dumps(report))
        raise
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
