import importlib.util
import io
import json
from pathlib import Path
from urllib.error import URLError
import unittest


MODULE_PATH = Path(__file__).resolve().parents[1] / "rk_server.py"
SPEC = importlib.util.spec_from_file_location("rk_server", MODULE_PATH)
rk_server = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(rk_server)


def live_payload():
    angle_min = -1.647495366
    angle_increment = 0.00306796166
    return {
        "schema_version": 1,
        "origin": "managed_candidate_not_factory_scan",
        "live": True,
        "status": "live",
        "source_age_s": 0.03,
        "scan": {
            "ranges": [1.0] * 1075,
            "geometry": {
                "angle_min": angle_min,
                "angle_max": angle_min + 1074 * angle_increment,
                "angle_increment": angle_increment,
                "range_min": 0.05,
                "range_max": 40.0,
                "scan_time": 1.0 / 15.0,
                "time_increment": (1.0 / 15.0) / 2048.0,
            },
        },
        "stats": {"point_count": 1075, "valid_return_count": 1075},
    }


class Response:
    def __init__(self, payload):
        self.buffer = io.BytesIO(json.dumps(payload).encode("utf-8"))

    def read(self, size=-1):
        return self.buffer.read(size)

    def __enter__(self):
        return self

    def __exit__(self, exception_type, exception, traceback):
        return False


class RkServerTest(unittest.TestCase):
    def test_fetches_only_fixed_upstream_and_validates_scan(self):
        calls = []

        def opener(request, timeout):
            calls.append((request.full_url, timeout))
            return Response(live_payload())

        status, payload = rk_server.fetch_upstream(opener=opener)
        self.assertEqual(200, status)
        self.assertTrue(payload["live"])
        self.assertEqual([(rk_server.UPSTREAM_URL, rk_server.UPSTREAM_TIMEOUT_SECONDS)], calls)

    def test_unavailable_upstream_returns_bounded_offline_status(self):
        def opener(request, timeout):
            raise URLError("unavailable")

        status, payload = rk_server.fetch_upstream(opener=opener)
        self.assertEqual(502, status)
        self.assertFalse(payload["live"])
        self.assertEqual("upstream_unavailable", payload["status"])
        self.assertIsNone(payload["scan"])

    def test_wrong_point_count_is_rejected(self):
        payload = live_payload()
        payload["scan"]["ranges"].pop()

        def opener(request, timeout):
            return Response(payload)

        status, result = rk_server.fetch_upstream(opener=opener)
        self.assertEqual(502, status)
        self.assertEqual("invalid_upstream", result["status"])

    def test_nonstandard_nonfinite_json_is_rejected(self):
        with self.assertRaises(ValueError):
            rk_server.strict_json_loads(b'{"range":NaN}')


if __name__ == "__main__":
    unittest.main()
