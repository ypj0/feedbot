"""Task-start localization confirmation via motion-accumulated AMCL convergence.

Replaces the former single-frame global-uniqueness gate. Confirmation requires
BOTH covariance convergence AND several consecutive dynamic-robust verification
frames. Motion is observed for diagnostics but is not a hard precondition: a
reliable prior may already converge while stationary; otherwise hand-push or
low-speed motion brings AMCL to convergence, which is reflected by the same gate.

This gate only reports `localization_ready`; it never authorizes motion.
"""
from localization_posterior import covariance_matrix


class ConvergeGate:
    CONFIRM_FRAMES = 3          # 0.2s verification cadence -> about 0.6s of consistency
    YAW_MAX_LOG = None          # convergence uses localization_posterior geometry, no extra tuning

    def __init__(self):
        self.phase = 'idle'
        self.reason = 'explicit_start_required'
        self.confirmed = 0
        self.motion_observed = False
        self.map_key = None
        self.source_key = None

    def start(self):
        if self.phase == 'running':
            raise ValueError('confirmation already running')
        self.phase = 'running'
        self.reason = 'accumulating_convergence_evidence'
        self.confirmed = 0
        self.motion_observed = False
        self.map_key = None
        self.source_key = None

    def update(self, verification, odometry_speeds):
        # verification: dict with phase, result, posterior(covariance 36), identity keys
        # odometry_speeds: iterable of twist components; any |v|>0.01 marks motion
        if self.phase == 'idle':
            return self.status()
        if self.phase == 'unlocalized':
            return self.status()
        self.motion_observed = self.motion_observed or any(
            abs(value) > .01 for value in odometry_speeds)
        phase = verification.get('phase')
        result = verification.get('result') or {}
        # Identity change resets the consecutive counter; never recycle old evidence.
        identity = (verification.get('map_generation'), verification.get('map_digest'),
                    verification.get('posterior_generation'), verification.get('posterior_source'))
        if self.map_key is None:
            self.map_key = identity
        elif identity != self.map_key:
            # A map or posterior-source change invalidates any prior confirmation;
            # it cannot silently keep the old ready state.
            self._lose('localization_source_changed')
            self.map_key = identity
            return self.status()

        compatible = (result.get('accepted') or
                      (result.get('pending_convergence') and
                       (result.get('innovation') or {}).get('compatible')))
        covariance = (verification.get('posterior') or {}).get('covariance')
        converged = self._converged(covariance)

        if phase in ('error', 'rejected', 'input_stale', 'waiting_map'):
            self._lose('verification_' + phase)
            return self.status()
        elif not converged:
            self.confirmed = 0
            self.reason = 'covariance_not_converged'
            # Preserve prior motion evidence; convergence may arrive after more motion.
        elif not compatible:
            self.confirmed = 0
            self.reason = 'verification_not_compatible'
        else:
            self.confirmed += 1
            self.reason = 'accumulating_convergence_evidence'
        if self.confirmed >= self.CONFIRM_FRAMES:
            self.phase = 'converged'
            self.reason = 'motion_accumulated_convergence_confirmed'
        return self.status()

    def _lose(self, reason):
        # Untracked move, rejected geometry, input failure, or identity change:
        # enter an honest unlocalized state and drop the prior confirmation.
        self.phase = 'unlocalized'
        self.reason = reason
        self.confirmed = 0

    @staticmethod
    def _converged(covariance):
        if covariance is None or len(covariance) != 36:
            return False
        # AMCL publishes 6x6 (row-major); x/y/yaw live at indices 0,7,35.
        sub = [[covariance[row * 6 + column] for column in (0, 1, 5)]
               for row in (0, 1, 5)]
        return covariance_matrix(sub, require_converged=True) is not None

    def status(self):
        return dict(phase=self.phase, reason=self.reason,
                    confirmed=self.confirmed, required=self.CONFIRM_FRAMES,
                    motion_observed=self.motion_observed,
                    localization_ready=self.phase == 'converged',
                    motion_authorized=False,
                    map_key=self.map_key, source_key=None)
