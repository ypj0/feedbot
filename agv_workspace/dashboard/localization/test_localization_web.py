import importlib.util
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen
from http.server import ThreadingHTTPServer


source = Path(__file__).with_name("car_bridge.py")
specification = importlib.util.spec_from_file_location("localization_web", source)
bridge = importlib.util.module_from_spec(specification)
specification.loader.exec_module(bridge)


class WebTests(unittest.TestCase):
    def current_state(self):
        state = bridge.State()
        state.set_map({'digest': 'current'}, 100)
        state.put('verification', dict(map_digest='current', map_generation=1,
                                      posterior_generation=1, posterior_source='amcl-a',
                                      observation_stamp_ns=200, phase='verified_pair',
                                      pair_verified=True, reason='verified', observation_age_s=.1,
                                      posterior=dict(stamp_ns=200, frame_id='map',
                                                     pose=[1., 2., .1], covariance=[.01] * 36)))
        for name in ('scan', 'robot_tf'):
            state.put(name, {})
        return state

    def test_current_tracking_changes_without_new_acquisition(self):
        state = self.current_state()
        self.assertTrue(state.snapshot()['tracking_valid'])
        self.assertFalse(state.snapshot()['motion_authorized'])
        verification = dict(state.values['verification'][1])
        for phase in ('pending_convergence', 'waiting_pair', 'rejected', 'input_stale', 'error'):
            state.put('verification', dict(verification, phase=phase, pair_verified=False))
            self.assertFalse(state.snapshot()['tracking_valid'])
        state.put('verification', verification)
        self.assertTrue(state.snapshot()['tracking_valid'])
        state.put('verification', verification, time.monotonic() - 1)
        self.assertFalse(state.snapshot()['tracking_valid'])

    def test_changed_map_invalidates_old_display_and_queued_verification(self):
        state = self.current_state()
        old = dict(state.values['verification'][1])
        state.set_map({'digest': 'new-map'}, 300)
        state.put('verification', old)
        result = state.snapshot()
        for name in ('amcl', 'robot_tf', 'scan', 'verification'):
            self.assertNotIn(name, result)
        self.assertFalse(result['tracking_valid'])
        self.assertFalse(result['data_ready'])

    def test_first_or_repeated_map_keeps_same_source_stationary_posterior(self):
        state = bridge.State()
        verification = dict(self.current_state().values['verification'][1])
        verification.update(phase='waiting_pair', pair_verified=False, observation_age_s=30.)
        state.put('verification', verification)
        state.set_map({'digest': 'current'}, 200)
        state.set_map({'digest': 'current'}, 300)
        self.assertIn('amcl', state.snapshot())
        self.assertEqual(state.snapshot()['amcl']['observation_age_s'], 30.)
        self.assertFalse(state.snapshot()['tracking_valid'])

    def test_source_change_cannot_display_old_posterior(self):
        state = self.current_state()
        state.put('verification', dict(map_digest='current', map_generation=1,
                                      posterior_generation=2, posterior_source='amcl-b',
                                      observation_stamp_ns=0, phase='waiting_pair', pair_verified=False,
                                      posterior=None))
        self.assertNotIn('amcl', state.snapshot())
        posterior = dict(stamp_ns=300, frame_id='map', pose=[3., 4., .2], covariance=[.01] * 36)
        state.put('verification', dict(map_digest='current', map_generation=1,
                                      posterior_generation=2, posterior_source='amcl-b',
                                      observation_stamp_ns=300, phase='pending_convergence',
                                      pair_verified=False, posterior=posterior, observation_age_s=.1))
        self.assertIn('amcl', state.snapshot())
        self.assertEqual(state.snapshot()['amcl']['x'], 3.)
        self.assertFalse(state.snapshot()['tracking_valid'])

    def test_new_posterior_cannot_borrow_previous_frames_verification(self):
        state = self.current_state()
        verification = dict(state.values['verification'][1])
        verification['posterior'] = dict(verification['posterior'], stamp_ns=250)
        state.put('verification', verification)
        self.assertFalse(state.snapshot()['tracking_valid'])
        self.assertNotIn('amcl', state.snapshot())

    def test_same_gid_new_generation_with_no_pair_clears_previous_display(self):
        state = self.current_state()
        verification = dict(state.values['verification'][1])
        state.put('verification', dict(verification, posterior_generation=2,
                                      observation_stamp_ns=0, pair_verified=False,
                                      phase='waiting_pair', posterior=None))
        self.assertNotIn('amcl', state.snapshot())
        self.assertFalse(state.snapshot()['tracking_valid'])

    def test_missing_or_stale_source_verification_hides_unbound_posterior(self):
        state = self.current_state()
        state.put('verification', state.values['verification'][1], time.monotonic() - 1)
        self.assertNotIn('amcl', state.snapshot())
        self.assertFalse(state.snapshot()['data_ready'])

    def scan_message(self, nanosec):
        return SimpleNamespace(header=SimpleNamespace(
            stamp=SimpleNamespace(sec=1, nanosec=nanosec), frame_id='scan'))

    def test_pending_scan_survives_new_arrival_until_exact_tf_ready(self):
        pairing = bridge.ScanPairing()
        first, second = self.scan_message(0), self.scan_message(70000000)
        pairing.offer(first, 10.)
        self.assertIsNone(pairing.take(10.02, 1020000000, lambda message: False))
        pairing.offer(second, 10.07)
        queried = []
        def available(message):
            queried.append(message.header.stamp.nanosec)
            return True
        self.assertEqual(pairing.take(10.08, 1080000000, available), (10., first))
        self.assertEqual(queried, [0])
        self.assertEqual(pairing.skipped, 1)
        self.assertIsNone(pairing.take(10.09, 1090000000, available))

    def test_pending_scan_expiry_is_not_extended_by_new_arrivals(self):
        pairing = bridge.ScanPairing()
        pairing.offer(self.scan_message(0), 10.)
        pairing.offer(self.scan_message(490000000), 10.49)
        with self.assertRaisesRegex(ValueError, 'expired'):
            pairing.take(10.501, 1501000000, lambda message: True)
        self.assertIsNone(pairing.pending)

    def test_pending_scan_checks_source_time_before_transform(self):
        for now_ns in (999999999, 1500000001):
            pairing = bridge.ScanPairing()
            pairing.offer(self.scan_message(0), 10.)
            with self.assertRaisesRegex(ValueError, 'source timestamp'):
                pairing.take(10.01, now_ns, lambda message: self.fail('invalid source queried TF'))
            self.assertIsNone(pairing.pending)

    def test_verification_is_bound_to_map_and_freshness(self):
        state = bridge.State()
        state.put('map', {'digest': 'current'})
        state.put('verification', {'map_digest': 'old'})
        self.assertNotIn('verification', state.snapshot())
        state.put('verification', {'map_digest': 'current'})
        self.assertIn('verification', state.snapshot())
        state.put('verification', {'map_digest': 'current'}, time.monotonic() - 1)
        self.assertNotIn('verification', state.snapshot())

    def test_source_timestamp_rejects_buffered_and_future_scan(self):
        now_ns = 2000000000
        bridge.require_current_stamp(SimpleNamespace(sec=1, nanosec=900000000), now_ns)
        for stamp in (SimpleNamespace(sec=1, nanosec=0),
                      SimpleNamespace(sec=2, nanosec=100000000)):
            with self.assertRaises(ValueError):
                bridge.require_current_stamp(stamp, now_ns)

    def test_no_motion_publisher_or_post_handler(self):
        self.assertNotIn("create_publisher", source.read_text())
        self.assertNotIn("cmd_vel", source.read_text())
        self.assertFalse(hasattr(bridge.Handler, "do_POST"))

    def test_stale_realtime_data_removed_but_map_retained(self):
        state = bridge.State()
        for name in ("map", "scan", "robot_tf", "chassis"):
            state.put(name, {}, time.monotonic() - 1)
        result = state.snapshot()
        self.assertIn("map", result)
        for name in ("scan", "robot_tf", "chassis"):
            self.assertNotIn(name, result)
            self.assertIn(name, result["errors"])
        self.assertFalse(result["data_ready"])

    def test_transform_failure_does_not_leave_old_pose(self):
        state = bridge.State()
        state.put("robot_tf", {"x": 1})
        state.fail("robot_tf", RuntimeError("TF missing"))
        self.assertNotIn("robot_tf", state.snapshot())
        self.assertEqual(state.snapshot()["errors"]["robot_tf"], "TF missing")

    def test_ready_is_not_localization_acceptance(self):
        state = self.current_state()
        self.assertTrue(state.snapshot()["data_ready"])
        self.assertFalse(state.snapshot()["localization_accepted"])

    def test_same_message_pose_stamp_covariance(self):
        covariance = [0.0] * 36
        covariance[35] = 0.11
        message = dict(observation_stamp_ns=7000000123, observation_age_s=.2,
                       posterior=dict(frame_id='map', stamp_ns=7000000123,
                                      pose=[2., 3., 0.], covariance=covariance))
        result = bridge.amcl_payload(message)
        self.assertEqual((result["x"], result["y"], result["stamp_ns"]), (2, 3, 7000000123))
        self.assertEqual(result["covariance"], covariance)
        self.assertAlmostEqual(result["yaw_std_deg"], 19.002859, places=4)
        covariance[35] = -1
        with self.assertRaises(ValueError):
            bridge.amcl_payload(message)

    def test_http_get_and_motion_endpoint_rejection(self):
        server = ThreadingHTTPServer(("127.0.0.1", 0), bridge.Handler)
        server.token, server.state = "fixture", bridge.State()
        worker = threading.Thread(target=server.serve_forever)
        worker.start()
        address = "http://127.0.0.1:%s" % server.server_port
        try:
            with urlopen(Request(address + "/api/state", headers={"X-Test-Token": "fixture"})) as response:
                self.assertFalse(json.load(response)["motion_api"])
            with self.assertRaises(HTTPError) as error:
                urlopen(Request(address + "/api/command", data=b"{}", headers={"X-Test-Token": "fixture"}))
            self.assertEqual(error.exception.code, 501)
            error.exception.close()
            with self.assertRaises(HTTPError) as error:
                urlopen(address + "/api/state")
            self.assertEqual(error.exception.code, 403)
            error.exception.close()
        finally:
            server.shutdown()
            server.server_close()
            worker.join()


if __name__ == "__main__":
    unittest.main()
