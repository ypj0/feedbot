import importlib.util
from pathlib import Path
import sys
import types
import unittest
from unittest.mock import patch


ROOT = Path(__file__).parent


class ChangeState:
    class Request:
        def __init__(self):
            self.transition = types.SimpleNamespace(id=None)


class GetState:
    class Request:
        pass


rclpy = types.ModuleType("rclpy")
rclpy.spin_until_future_complete = lambda node, future, timeout_sec: None
lifecycle_msgs = types.ModuleType("lifecycle_msgs")
lifecycle_msgs_srv = types.ModuleType("lifecycle_msgs.srv")
lifecycle_msgs_srv.ChangeState = ChangeState
lifecycle_msgs_srv.GetState = GetState
SPEC = importlib.util.spec_from_file_location("activate_mapping", ROOT / "activate_mapping.py")
ACTIVATION = importlib.util.module_from_spec(SPEC)
with patch.dict(sys.modules, {"rclpy": rclpy, "lifecycle_msgs": lifecycle_msgs,
                              "lifecycle_msgs.srv": lifecycle_msgs_srv}):
    SPEC.loader.exec_module(ACTIVATION)


class Future:
    def __init__(self, result):
        self.value = result

    def done(self):
        return True

    def result(self):
        return self.value


class Client:
    def __init__(self, events, service, responses):
        self.events = events
        self.service = service
        self.responses = responses

    def wait_for_service(self, timeout_sec):
        self.events.append(("wait", self.service, timeout_sec))
        return True

    def call_async(self, request):
        identifier = getattr(getattr(request, "transition", None), "id", None)
        self.events.append(("call", self.service, identifier))
        return Future(self.responses.pop(0))


class Node:
    def __init__(self, responses):
        self.events = []
        self.responses = list(responses)

    def create_client(self, service_type, service):
        return Client(self.events, service, self.responses)


class MappingActivationTest(unittest.TestCase):
    def test_configure_then_activate_with_confirmed_states(self):
        node = Node([
            types.SimpleNamespace(success=True),
            types.SimpleNamespace(current_state=types.SimpleNamespace(id=2)),
            types.SimpleNamespace(success=True),
            types.SimpleNamespace(current_state=types.SimpleNamespace(id=3)),
        ])
        report = ACTIVATION.activate_mapping(node)
        calls = [event for event in node.events if event[0] == "call"]
        self.assertEqual(calls, [
            ("call", ACTIVATION.CHANGE_STATE_SERVICE, 1),
            ("call", ACTIVATION.GET_STATE_SERVICE, None),
            ("call", ACTIVATION.CHANGE_STATE_SERVICE, 3),
            ("call", ACTIVATION.GET_STATE_SERVICE, None),
        ])
        self.assertTrue(report["passed"])
        self.assertEqual([item["state"] for item in report["transitions"]], [2, 3])

    def test_configure_failure_does_not_continue(self):
        node = Node([types.SimpleNamespace(success=False)])
        with self.assertRaisesRegex(RuntimeError, "configure transition failed"):
            ACTIVATION.activate_mapping(node)
        calls = [event for event in node.events if event[0] == "call"]
        self.assertEqual(calls, [("call", ACTIVATION.CHANGE_STATE_SERVICE, 1)])

    def test_wrong_configured_state_does_not_activate(self):
        node = Node([
            types.SimpleNamespace(success=True),
            types.SimpleNamespace(current_state=types.SimpleNamespace(id=1)),
        ])
        report = {"passed": False, "node": "/slam_toolbox", "transitions": []}
        with self.assertRaisesRegex(RuntimeError, "state after configure"):
            ACTIVATION.activate_mapping(node, report)
        calls = [event for event in node.events if event[0] == "call"]
        self.assertEqual(calls, [
            ("call", ACTIVATION.CHANGE_STATE_SERVICE, 1),
            ("call", ACTIVATION.GET_STATE_SERVICE, None),
        ])
        self.assertFalse(report["passed"])
        self.assertEqual(report["transitions"], [])


if __name__ == "__main__":
    unittest.main()
