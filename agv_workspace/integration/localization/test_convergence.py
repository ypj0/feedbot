import copy
import unittest

from convergence import Convergence


def observation(phase='waiting_pair', stamp=100):
    return dict(error=None, map_key=(1, 'map-a'), source_key=(1, 'amcl-first'), phase=phase,
                observation_stamp_ns=stamp, scan_stamp_ns=stamp + 1,
                result={'accepted': False, 'pending_convergence': True,
                        'innovation': {'compatible': True}},
                counters=(0, 0, 0, 0))


class ConvergenceTests(unittest.TestCase):
    def test_acquisition_continues_beyond_ten_and_stops_on_verified_pair(self):
        state = Convergence()
        view = observation('pending_convergence')
        state.start(0., view, operation='acquire')
        for index in range(15):
            self.assertTrue(state.step(index * 1.01, view))
            state.response(True)
            phase = 'verified_pair' if index == 14 else 'pending_convergence'
            view = observation(phase, 200 + index * 100)
            self.assertFalse(state.step(index * 1.01 + .2, view))
        self.assertEqual(state.phase, 'succeeded')
        self.assertEqual(state.requests, 15)
        self.assertEqual(state.confirmed, 15)
        self.assertFalse(state.step(20., view))
        self.assertFalse(state.status()['localization_accepted'])
        self.assertEqual(state.status()['operation'], 'acquire')
        self.assertEqual(state.status()['budget_s'], 30.)

    def test_acquisition_deadline_includes_pending_request_and_no_restart(self):
        state = Convergence()
        view = observation('pending_convergence')
        state.start(0., view, operation='acquire')
        for index in range(29):
            self.assertTrue(state.step(index * 1.02, view))
            state.response(True)
            view = observation('pending_convergence', 200 + index * 100)
            state.step(index * 1.02 + .2, view)
        self.assertTrue(state.step(29.9, view))
        state.response(True)
        self.assertFalse(state.step(30., observation('verified_pair', 4000)))
        self.assertEqual(state.reason, 'budget_exhausted')
        self.assertEqual(state.requests, 30)
        self.assertFalse(state.step(31., view))
        self.assertEqual(state.requests, 30)

    def test_operation_cannot_change_while_running_and_refresh_resets_budget(self):
        state = Convergence()
        state.start(0., observation(), operation='acquire')
        with self.assertRaises(ValueError):
            state.start(1., observation())
        self.assertEqual(state.deadline, 30.)
        state.fail('explicit_stop')
        state.start(2., observation())
        self.assertEqual(state.deadline, 12.)
        self.assertEqual(state.status()['operation'], 'refresh')
        self.assertEqual(state.status()['budget_s'], 10.)

    def test_acquisition_preserves_fault_source_map_and_timeout_failures(self):
        for changed in (dict(error='nonstationary_wheel'), dict(map_key=(2, 'changed')),
                        dict(source_key=(2, 'changed')), dict(counters=(1, 0, 0, 0)),
                        dict(phase='rejected'), dict(phase='input_stale')):
            state = Convergence()
            view = observation()
            state.start(0., view, operation='acquire')
            state.step(0., view)
            self.assertFalse(state.step(.2, dict(view, **changed)))
            self.assertEqual(state.phase, 'failed')
            self.assertFalse(state.step(2., view))
            self.assertEqual(state.requests, 1)
        for acknowledged in (True, False):
            state = Convergence()
            view = observation()
            state.start(0., view, operation='acquire')
            state.step(0., view)
            if acknowledged:
                state.response(True)
            state.step(1., view)
            self.assertEqual(state.reason, 'new_pair_timeout' if acknowledged else 'service_response_timeout')
            self.assertFalse(state.step(2., view))
            self.assertEqual(state.requests, 1)

    def test_invalid_operation_is_not_a_configurable_budget(self):
        state = Convergence()
        with self.assertRaises(ValueError):
            state.start(0., observation(), operation='unlimited')
        self.assertEqual(state.phase, 'idle')

    def test_request_boundary_is_visible_and_equal_scan_is_new_posterior(self):
        state = Convergence()
        self.assertIsNone(state.status()['request_sent_monotonic_s'])
        self.assertIsNone(state.status()['request_scan_stamp_ns'])
        self.assertIsNone(state.status()['request_posterior_stamp_ns'])
        view = observation()
        view['scan_stamp_ns'] = 300
        state.start(0., view)
        self.assertTrue(state.step(.1, view))
        state.response(True)
        status = state.status()
        self.assertEqual(status['request_scan_stamp_ns'], 300)
        self.assertEqual(status['request_posterior_stamp_ns'], 100)
        self.assertEqual(status['request_sent_monotonic_s'], .1)
        self.assertTrue(status['request_acknowledged'])
        self.assertTrue(status['request_pending'])
        self.assertFalse(state.step(.4, observation('verified_pair', 300)))
        self.assertEqual(state.confirmed, 1)
        self.assertFalse(state.step(1.11, observation('verified_pair', 300)))
        self.assertEqual(state.reason, 'new_pair_verified')
        self.assertEqual(state.status()['request_scan_stamp_ns'], 300)
        self.assertEqual(state.requests, 1)

    def test_boundary_status_does_not_reset_on_explicit_failed_start(self):
        state = Convergence()
        view = observation()
        state.start(0., view)
        state.step(.1, view)
        state.response(False)
        with self.assertRaises(ValueError):
            state.start(2., dict(view, error='missing input'))
        self.assertEqual(state.status()['request_sent_monotonic_s'], .1)
        state.start(3., view)
        self.assertIsNone(state.status()['request_scan_stamp_ns'])
        self.assertIsNone(state.status()['request_sent_monotonic_s'])

    def test_old_compatible_posterior_gets_one_refresh_not_authority(self):
        state = Convergence()
        view = observation()
        state.start(0., view)
        self.assertTrue(state.step(0., view))
        state.response(True)
        self.assertFalse(state.step(.5, view))
        self.assertEqual(state.phase, 'running')
        self.assertFalse(state.status()['localization_accepted'])

    def test_ack_alone_times_out_without_retry(self):
        state = Convergence()
        view = observation()
        state.start(0., view)
        state.step(0., view)
        state.response(True)
        self.assertFalse(state.step(1., view))
        self.assertEqual(state.reason, 'new_pair_timeout')
        self.assertFalse(state.step(2., view))
        self.assertEqual(state.requests, 1)

    def test_new_pair_and_response_are_both_required(self):
        state = Convergence()
        view = observation()
        state.start(0., view)
        state.step(0., view)
        newer = observation('verified_pair', 200)
        self.assertFalse(state.step(.3, newer))
        self.assertEqual(state.phase, 'running')
        state.response(True)
        self.assertFalse(state.step(.4, newer))
        self.assertEqual(state.phase, 'succeeded')
        self.assertEqual(state.requests, 1)

    def test_interval_distinct_scans_and_success_stop(self):
        state = Convergence()
        view = observation('pending_convergence')
        state.start(0., view)
        self.assertTrue(state.step(0., view))
        state.response(True)
        newer = observation('pending_convergence', 200)
        self.assertFalse(state.step(.3, newer))
        self.assertFalse(state.step(.99, newer))
        self.assertTrue(state.step(1.01, newer))
        state.response(True)
        self.assertFalse(state.step(1.2, newer))
        final = observation('verified_pair', 300)
        self.assertFalse(state.step(1.3, final))
        self.assertEqual(state.phase, 'succeeded')
        self.assertFalse(state.step(3., final))
        self.assertEqual(state.requests, 2)

    def test_deadline_includes_service_wait(self):
        state = Convergence()
        view = observation()
        state.start(0., view)
        self.assertFalse(state.step(10., view))
        self.assertEqual(state.reason, 'budget_exhausted')
        self.assertEqual(state.requests, 0)

    def test_ten_requests_is_terminal_not_auto_restart(self):
        state = Convergence()
        view = observation('pending_convergence')
        state.start(0., view)
        for index in range(10):
            self.assertTrue(state.step(index * 1.01, view))
            state.response(True)
            view = observation('pending_convergence', 200 + index * 100)
            self.assertFalse(state.step(index * 1.01 + .2, view))
        self.assertEqual(state.requests, 10)
        self.assertEqual(state.reason, 'budget_exhausted')
        self.assertFalse(state.step(30., view))

    def test_inputs_map_motion_or_fault_change_stop(self):
        for field, value in (('error', 'nonstationary'), ('map_key', (2, 'map-b')),
                             ('counters', (1, 0, 0, 0)), ('phase', 'input_stale')):
            with self.subTest(field=field):
                state = Convergence()
                view = observation()
                state.start(0., view)
                state.step(0., view)
                changed = copy.deepcopy(view)
                changed[field] = value
                self.assertFalse(state.step(.2, changed))
                self.assertEqual(state.phase, 'failed')
                state.response(True)
                self.assertEqual(state.phase, 'failed')

    def test_rejection_and_service_failure_are_terminal(self):
        for fail_service in (True, False):
            state = Convergence()
            view = observation()
            state.start(0., view)
            state.step(0., view)
            state.response(not fail_service)
            state.step(.3, observation('rejected', 200))
            self.assertEqual(state.phase, 'failed')
            self.assertFalse(state.step(2., view))

    def test_invalid_start_and_duplicate_start_do_not_send(self):
        for view in (observation('rejected'), dict(observation(), error='missing input'),
                     dict(observation(), result=None)):
            with self.assertRaises(ValueError):
                Convergence().start(0., view)
        state = Convergence()
        state.start(0., observation())
        with self.assertRaises(ValueError):
            state.start(.1, observation())
        self.assertEqual(state.requests, 0)

    def test_current_verified_pair_still_requires_requested_new_scan(self):
        state = Convergence()
        state.start(0., observation('verified_pair'))
        self.assertEqual(state.phase, 'running')
        self.assertTrue(state.step(0., observation('verified_pair')))
        state.response(True)
        self.assertFalse(state.step(.2, observation('verified_pair')))
        self.assertEqual(state.phase, 'running')
        self.assertFalse(state.step(.3, observation('verified_pair', 200)))
        self.assertEqual(state.phase, 'succeeded')
        self.assertEqual(state.requests, 1)

    def test_unseen_fresh_posterior_progress_is_independent_of_scan_delivery(self):
        state = Convergence()
        view = observation()
        view['scan_stamp_ns'] = 300
        state.start(0., view)
        state.step(0., view)
        state.response(True)
        self.assertFalse(state.step(.3, observation('verified_pair', 200)))
        self.assertEqual(state.phase, 'succeeded')
        self.assertEqual(state.confirmed, 1)

    def test_amcl_replacement_stops_pending_request_without_retry(self):
        state = Convergence()
        view = observation()
        state.start(0., view)
        self.assertTrue(state.step(0., view))
        state.response(True)
        replaced = observation('verified_pair', 200)
        replaced['source_key'] = (2, 'amcl-second')
        self.assertFalse(state.step(.2, replaced))
        self.assertEqual(state.phase, 'failed')
        self.assertEqual(state.reason, 'posterior_source_changed')
        self.assertFalse(state.step(2., replaced))
        self.assertEqual(state.requests, 1)


if __name__ == '__main__':
    unittest.main()
