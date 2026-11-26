import unittest
from converge_gate import ConvergeGate


def cov(scale=0.05):
    # small positive-definite diagonal x/y/yaw covariance
    c = [0.0] * 36
    for i in (0, 7, 35):
        c[i] = scale * scale
    return c


def verified(phase='verified_pair', accepted=True, conv=True, scale=0.05,
             gen=(1, 'map-a', 1, 'amcl-a')):
    result = {'accepted': accepted} if phase == 'verified_pair' else \
             {'pending_convergence': True, 'accepted': False,
              'innovation': {'compatible': accepted}}
    return dict(phase=phase, result=result, posterior={'covariance': cov(scale) if conv else [0]*36},
                map_generation=gen[0], map_digest=gen[1],
                posterior_generation=gen[2], posterior_source=gen[3])


class ConvergeGateTests(unittest.TestCase):
    def test_requires_three_converged_compatible_frames(self):
        gate = ConvergeGate()
        gate.start()
        self.assertFalse(gate.update(verified(), [0, 0, 0])['localization_ready'])
        self.assertFalse(gate.update(verified(), [0, 0, 0])['localization_ready'])
        status = gate.update(verified(), [0, 0, 0])
        self.assertTrue(status['localization_ready'])
        self.assertEqual(status['phase'], 'converged')
        self.assertFalse(status['motion_authorized'])

    def test_unconverged_covariance_breaks_consecutive_run(self):
        gate = ConvergeGate()
        gate.start()
        gate.update(verified(), [0, 0, 0])
        gate.update(verified(), [0, 0, 0])
        # non-converged covariance resets
        gate.update(verified(conv=False), [0, 0, 0])
        self.assertFalse(gate.update(verified(), [0, 0, 0])['localization_ready'])

    def test_rejected_frame_resets(self):
        gate = ConvergeGate()
        gate.start()
        gate.update(verified(), [0, 0, 0])
        gate.update(verified(), [0, 0, 0])
        gate.update(verified(phase='rejected', accepted=False), [0, 0, 0])
        self.assertFalse(gate.update(verified(), [0, 0, 0])['localization_ready'])

    def test_identity_change_resets_and_never_recycles_old_frames(self):
        gate = ConvergeGate()
        gate.start()
        gate.update(verified(), [0, 0, 0])
        gate.update(verified(), [0, 0, 0])
        gate.update(verified(gen=(2, 'map-b', 1, 'amcl-a')), [0, 0, 0])
        self.assertFalse(gate.update(verified(gen=(2, 'map-b', 1, 'amcl-a')), [0, 0, 0])['localization_ready'])

    def test_motion_is_observed_as_diagnostic_not_precondition(self):
        gate = ConvergeGate()
        gate.start()
        # stationary frames can still converge
        for _ in range(3):
            gate.update(verified(), [0, 0, 0])
        self.assertTrue(gate.status()['localization_ready'])
        self.assertFalse(gate.status()['motion_observed'])

    def test_no_start_means_idle(self):
        gate = ConvergeGate()
        self.assertEqual(gate.update(verified(), [0, 0, 0])['phase'], 'idle')
        self.assertFalse(gate.status()['localization_ready'])


if __name__ == '__main__':
    unittest.main()


class ConvergeGateLoseTests(unittest.TestCase):
    def _converge(self):
        gate = ConvergeGate()
        gate.start()
        for _ in range(3):
            gate.update(verified(), [0, 0, 0])
        self.assertTrue(gate.status()['localization_ready'])
        return gate

    def test_rejected_after_converged_enters_unlocalized(self):
        gate = self._converge()
        gate.update(verified(phase='rejected', accepted=False), [0, 0, 0])
        status = gate.status()
        self.assertEqual(status['phase'], 'unlocalized')
        self.assertFalse(status['localization_ready'])

    def test_identity_change_after_converged_enters_unlocalized(self):
        gate = self._converge()
        gate.update(verified(gen=(2, 'map-b', 1, 'amcl-a')), [0, 0, 0])
        self.assertEqual(gate.status()['phase'], 'unlocalized')
        self.assertFalse(gate.status()['localization_ready'])

    def test_unlocalized_stays_unlocalized_until_restart(self):
        gate = self._converge()
        gate.update(verified(phase='rejected', accepted=False), [0, 0, 0])
        # 之后即使来了好帧，也不得自动恢复
        for _ in range(5):
            gate.update(verified(), [0, 0, 0])
        self.assertEqual(gate.status()['phase'], 'unlocalized')
        self.assertFalse(gate.status()['localization_ready'])

    def test_restart_after_unlocalized_can_reconverge(self):
        gate = self._converge()
        gate.update(verified(phase='rejected', accepted=False), [0, 0, 0])
        gate.start()
        for _ in range(3):
            gate.update(verified(), [0, 0, 0])
        self.assertTrue(gate.status()['localization_ready'])

    def test_error_input_after_converged_enters_unlocalized(self):
        gate = self._converge()
        gate.update(verified(phase='error', accepted=False), [0, 0, 0])
        self.assertEqual(gate.status()['phase'], 'unlocalized')
        self.assertFalse(gate.status()['localization_ready'])
