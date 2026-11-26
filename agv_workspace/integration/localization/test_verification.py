import copy
from pathlib import Path
import unittest
from unittest.mock import patch

import numpy as np

from test_global_localization import fixture
from verification import Verification, evaluate


class VerificationTests(unittest.TestCase):
    def setUp(self):
        self.state = Verification()
        self.state.set_posterior_source('amcl-first', 0)
        self.grid = dict(width=2, height=2, resolution=.05, origin=[0., 0., 0.], data=[0] * 4)
        self.covariance = np.eye(6).reshape(-1).tolist()

    def pair(self, stamp=1000000000):
        self.state.scan(stamp, [[1., 0.]])
        self.state.posterior(stamp, 'map', [1., 2., .1], self.covariance)

    def test_existing_posterior_is_adopted_without_sending_seed(self):
        self.state.set_map(self.grid, 1000000000)
        self.pair()
        job = self.state.prepare(1100000000)
        self.assertEqual(job['pose'], [1., 2., .1])
        self.assertEqual(job['covariance'], [[1., 0., 0.], [0., 1., 0.], [0., 0., 1.]])
        with patch('verification.search', return_value={'accepted': False}) as search:
            evaluate(job)
        self.assertEqual(search.call_args.args[2], [1., 2., .1])

    def test_completed_result_exports_original_full_posterior_atomically(self):
        self.state.set_map(self.grid, 0)
        self.pair()
        job = self.state.prepare(1100000000)
        self.pair(1200000000)
        self.state.complete(job, {'accepted': True, 'pose': [9., 9., 9.]})
        output = self.state.snapshot(1300000000)
        self.assertEqual(output['posterior']['stamp_ns'], output['observation_stamp_ns'])
        self.assertEqual(output['posterior']['pose'], [1., 2., .1])
        self.assertEqual(output['posterior']['covariance'], self.covariance)
        self.assertEqual(output['posterior']['frame_id'], 'map')
        self.assertEqual(output['observation_age_s'], .3)
        self.state.set_posterior_source('replacement', 1400000000)
        self.assertIsNone(self.state.snapshot(1500000000)['posterior'])

    def test_changed_map_clears_completed_posterior_and_rejects_old_job(self):
        self.state.set_map(self.grid, 0)
        self.pair()
        job = self.state.prepare(1100000000)
        self.state.complete(job, {'accepted': True})
        changed = copy.deepcopy(self.grid)
        changed['data'][0] = 100
        self.state.set_map(changed, 1200000000)
        self.assertFalse(self.state.complete(job, {'accepted': True}))
        self.assertIsNone(self.state.snapshot(1300000000)['posterior'])

    def test_first_map_arriving_late_preserves_current_pair(self):
        self.pair()
        self.assertIsNone(self.state.prepare(1100000000))
        self.state.set_map(self.grid, 1100000000)
        self.assertIsNotNone(self.state.prepare(1200000000))

    def test_different_stamps_never_pair(self):
        self.state.set_map(self.grid, 0)
        self.state.scan(1000000000, [[1., 0.]])
        self.state.posterior(1000000001, 'map', [1., 2., .1], self.covariance)
        self.assertIsNone(self.state.prepare(1100000000))

    def test_stale_or_future_cached_posterior_never_initializes(self):
        self.state.set_map(self.grid, 0)
        self.pair()
        self.assertIsNone(self.state.prepare(3000000000))
        self.assertIsNone(self.state.prepare(999999999))

    def test_map_change_rejects_old_jobs_and_late_old_pair(self):
        self.state.set_map(self.grid, 0)
        self.pair()
        job = self.state.prepare(1100000000)
        changed = copy.deepcopy(self.grid)
        changed['data'][0] = 100
        self.state.set_map(changed, 1200000000)
        self.assertFalse(self.state.complete(job, {'accepted': True}))
        self.pair()
        self.assertIsNone(self.state.prepare(1300000000))
        self.assertFalse(self.state.snapshot(1300000000)['pair_verified'])

    def test_repeated_same_map_and_duplicate_pair_not_reprocessed(self):
        self.state.set_map(self.grid, 0)
        self.pair()
        job = self.state.prepare(1100000000)
        self.state.set_map(copy.deepcopy(self.grid), 1200000000)
        self.assertEqual(self.state.generation, job['generation'])
        self.assertIsNone(self.state.prepare(1300000000))

    def test_verified_pair_is_never_navigation_authority(self):
        self.state.set_map(self.grid, 0)
        self.pair()
        job = self.state.prepare(1100000000)
        self.state.complete(job, {'accepted': True})
        current = self.state.snapshot(1200000000)
        self.assertTrue(current['pair_verified'])
        self.assertFalse(current['localization_accepted'])
        self.state.scan(3100000000, [[1., 0.]])
        stale = self.state.snapshot(3200000000)
        self.assertEqual(stale['phase'], 'waiting_pair')
        self.assertFalse(stale['pair_verified'])

    def test_completed_job_does_not_erase_input_fault(self):
        self.state.set_map(self.grid, 0)
        self.pair()
        job = self.state.prepare(1100000000)
        self.state.errors['scan'] = 'source timestamp invalid'
        self.state.complete(job, {'accepted': True})
        self.assertEqual(self.state.snapshot(1200000000)['phase'], 'error')

    def test_invalid_posterior_is_explicit(self):
        with self.assertRaises(ValueError):
            self.state.posterior(1, 'odom', [1., 2., .1], self.covariance)
        with self.assertRaises(ValueError):
            self.state.posterior(1, 'map', [1., 2., .1], [0.])

    def test_reused_geometry_rejects_wrong_pose_and_broad_covariance(self):
        grid, points, pose = fixture()
        job = dict(grid=grid, points=points, pose=pose, covariance=np.eye(3) * .001)
        self.assertTrue(evaluate(job)['accepted'])
        job['pose'] = [pose[0] + 1, pose[1], pose[2]]
        self.assertFalse(evaluate(job)['accepted'])
        job['pose'] = pose
        job['covariance'] = np.eye(3) * .1
        self.assertFalse(evaluate(job)['accepted'])

    def test_no_motion_initialization_or_service_client(self):
        source = Path(__file__).with_name('observer.py').read_text()
        for forbidden in ('initialpose', 'cmd_vel', 'create_client', 'reinitialize_global'):
            self.assertNotIn(forbidden, source)

    def test_publisher_loss_invalidates_result_and_inflight_job(self):
        self.state.set_posterior_source('amcl-first', 0)
        self.state.set_map(self.grid, 0)
        self.pair()
        job = self.state.prepare(1100000000)
        self.state.complete(job, {'accepted': True})
        self.state.set_posterior_source(None, 1200000000)
        self.state.scan(1250000000, [[1., 0.]])
        self.assertFalse(self.state.complete(job, {'accepted': True}))
        self.assertFalse(self.state.snapshot(1300000000)['pair_verified'])
        self.assertIsNone(self.state.result)

    def test_same_map_publisher_replacement_requires_new_pair(self):
        self.state.set_posterior_source('amcl-first', 0)
        self.state.set_map(self.grid, 0)
        self.pair()
        job = self.state.prepare(1100000000)
        self.state.complete(job, {'accepted': True})
        self.state.set_posterior_source('amcl-second', 1200000000)
        self.state.set_map(self.grid, 1200000000)
        self.pair()
        self.assertIsNone(self.state.prepare(1300000000))
        self.assertIsNone(self.state.result)
        self.pair(1400000000)
        new_job = self.state.prepare(1500000000)
        self.assertTrue(self.state.complete(new_job, {'accepted': True}))
        self.assertTrue(self.state.snapshot(1500000000)['pair_verified'])

    def test_first_publisher_adopts_retained_pair_without_restart(self):
        self.state = Verification()
        self.state.set_posterior_source(None, 0)
        self.state.set_posterior_source('amcl-first', 1100000000)
        self.state.set_map(self.grid, 1100000000)
        self.pair()
        job = self.state.prepare(1200000000)
        self.assertIsNotNone(job)
        self.state.set_posterior_source('amcl-first', 1250000000)
        self.assertTrue(self.state.complete(job, {'accepted': True}))


if __name__ == '__main__':
    unittest.main()
