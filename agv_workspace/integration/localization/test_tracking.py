import unittest

from tracking import Tracking


def view(stamp=1000000000, phase='verified_pair', **changes):
    value = dict(error=None, map_key=(1, 'map'), source_key=(1, 'amcl'),
                 phase=phase, observation_stamp_ns=stamp, counters=(0, 0, 0, 0),
                 result=dict(accepted=phase == 'verified_pair'))
    value.update(changes)
    return value


class TrackingTests(unittest.TestCase):
    def test_current_posterior_is_accepted_without_a_local_acquire(self):
        state = Tracking()
        state.update(1000000000, dict(error='missing_or_stale_scan'))
        status = state.update(1100000000, view())
        self.assertTrue(status['localization_ready'])
        self.assertFalse(status['motion_authorized'])

    def test_waiting_pending_rejected_and_new_valid_posterior(self):
        state = Tracking()
        self.assertTrue(state.update(1100000000, view())['localization_ready'])
        for phase in ('waiting_pair', 'pending_convergence', 'rejected'):
            self.assertFalse(state.update(1200000000, view(phase=phase))['localization_ready'])
        self.assertTrue(state.update(1400000000, view(1300000000))['localization_ready'])

    def test_heartbeat_does_not_refresh_old_or_future_source_time(self):
        state = Tracking()
        self.assertFalse(state.update(3000000000, view())['localization_ready'])
        self.assertFalse(state.update(3000000000, view(3100000000))['localization_ready'])
        self.assertTrue(state.update(3200000000, view(3150000000))['localization_ready'])

    def test_sensor_fault_recovery_requires_post_fault_posterior(self):
        state = Tracking()
        state.update(1100000000, view())
        broken = state.update(1200000000, dict(error='missing_or_stale_imu'))
        self.assertEqual(broken['phase'], 'invalid')
        self.assertFalse(state.update(1300000000, view())['localization_ready'])
        self.assertTrue(state.update(1400000000, view(1350000000))['localization_ready'])

    def test_map_and_source_changes_invalidate_previous_epoch(self):
        for changes in (dict(map_key=(2, 'new-map')), dict(source_key=(2, 'new-amcl'))):
            state = Tracking()
            state.update(1100000000, view())
            self.assertFalse(state.update(1200000000, view(**changes))['localization_ready'])
            self.assertFalse(state.update(1300000000, view(**changes))['localization_ready'])
            self.assertTrue(state.update(1400000000, view(1350000000, **changes))['localization_ready'])

    def test_nonzero_commands_are_not_sensor_faults_but_fault_counters_are(self):
        state = Tracking()
        state.update(1100000000, view())
        self.assertTrue(state.update(1200000000, view(counters=(4, 0, 0, 0)))['localization_ready'])
        self.assertFalse(state.update(1300000000, view(counters=(4, 1, 0, 0)))['localization_ready'])
        self.assertFalse(state.update(1400000000, view(counters=(4, 1, 0, 0)))['localization_ready'])
        self.assertTrue(state.update(1500000000, view(1450000000, counters=(4, 1, 0, 0)))['localization_ready'])

    def test_running_acquisition_and_inconsistent_result_do_not_authorize(self):
        state = Tracking()
        self.assertFalse(state.update(1100000000, view(), acquiring=True)['localization_ready'])
        self.assertFalse(state.update(1200000000, view(result={'accepted': False}))['localization_ready'])
        self.assertTrue(state.update(1300000000, view(1250000000))['localization_ready'])


if __name__ == '__main__':
    unittest.main()
