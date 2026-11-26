import math
import io
import sys
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import patch

sys.path.insert(0, str(Path(__file__).resolve().parent))
import comparison_probe as probe
import comparison_trial as trial
import verify_comparison as verify


class ComparisonTests(unittest.TestCase):
    def test_corrected_geometry(self):
        increment = 2 * math.pi / 2048
        scan = SimpleNamespace(ranges=[1.] * 1075, intensities=[2.] * 1075,
                               angle_min=-537 * increment, angle_max=537 * increment, angle_increment=increment)
        self.assertAlmostEqual(probe.geometry(scan, True)["endpoint_error_rad"], 0)

    def test_official_bias_recorded_not_copied(self):
        scan = SimpleNamespace(ranges=[1.] * 1075, intensities=[2.] * 1075,
                               angle_min=-1.65, angle_max=1.65, angle_increment=2 * math.pi / 2048)
        self.assertGreater(probe.geometry(scan, False)["endpoint_error_rad"], 0.005)
        with self.assertRaises(ValueError):
            probe.geometry(scan, True)

    def test_restore_failure_invalidates_successful_data(self):
        self.assertEqual(trial.verdict(True, False), "对照未完成，恢复异常")
        self.assertEqual(trial.verdict(False, False), "对照未完成，恢复异常")
        self.assertEqual(trial.verdict(False, True), "对照未完成，原厂已恢复")
        self.assertEqual(trial.verdict(True, True), "实车静态通过（雷达对照）")

    def test_pair_digest_preserves_beam_order(self):
        first = {"ranges": [1., 2.], "intensities": [4., 5.]}
        shifted = {"ranges": [1., 2.], "intensities": [5., 4.]}
        self.assertNotEqual(verify.beam_digest(first), verify.beam_digest(shifted))
        self.assertEqual(verify.beam_digest({"ranges": [None], "intensities": [0]}),
                         verify.beam_digest({"ranges": ["inf"], "intensities": [0]}))

    def test_observer_reports_malformed_vectors(self):
        observer = probe.Observer(True, io.StringIO())
        with patch.object(probe, "monotonic", return_value=1.):
            observer.receive(SimpleNamespace(ranges=[1], intensities=[]))
        self.assertIn("1075", observer.error)


if __name__ == "__main__":
    unittest.main()
