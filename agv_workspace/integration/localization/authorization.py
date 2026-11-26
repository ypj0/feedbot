"""Navigation authorization state machine.

This is a separate authority from localization correctness. localization_ready
is a necessary but not sufficient input. Motion release requires an explicit
operator/task request while localization is current, and is revoked on any
unlocalized/rejected/fault/stop/cancel evidence. It never reuses an old pose,
never auto-retries, and never authorizes motion by itself.

States:
  idle        no request, no authorization
  requested   explicit authorize request pending a current localization signal
  authorized  current localization + explicit request; navigation release edge
  revoked     authorization removed by fault/cancel/unlocalized; needs a new
              explicit request
"""
class NavigationAuthorization:
    def __init__(self):
        self.state = 'idle'
        self.reason = 'no_request'
        self.request_stamp_ns = None
        self.authorized_at_ns = None

    def request(self, now_ns, localization_ready):
        if self.state == 'authorized':
            raise ValueError('authorization already granted; explicit revoke required first')
        if not localization_ready:
            self.state = 'idle'
            self.reason = 'request_requires_current_localization'
            self.request_stamp_ns = None
            return self.status()
        self.state = 'requested'
        self.reason = 'awaiting_current_localization_signal'
        self.request_stamp_ns = now_ns
        return self.status()

    def update(self, localization_ready, fault=False, stop=False, cancel=False):
        # Fault/stop/cancel always revoke; they are never suppressed by localization.
        if fault:
            self.state = 'revoked'
            self.reason = 'fault'
            return self.status()
        if stop:
            self.state = 'revoked'
            self.reason = 'stop'
            return self.status()
        if cancel:
            self.state = 'idle'
            self.reason = 'cancelled'
            self.request_stamp_ns = None
            return self.status()
        if not localization_ready:
            # Lost localization removes any authorization; it cannot hold the old grant.
            if self.state in ('authorized', 'requested'):
                self.state = 'revoked'
                self.reason = 'unlocalized'
            return self.status()
        if self.state == 'requested':
            self.state = 'authorized'
            self.reason = 'current_localization_and_explicit_request'
            self.authorized_at_ns = self.request_stamp_ns
        return self.status()

    def revoke(self, reason='operator_revoke'):
        self.state = 'revoked'
        self.reason = reason

    def status(self):
        return dict(state=self.state, reason=self.reason,
                    navigation_authorized=self.state == 'authorized',
                    request_stamp_ns=self.request_stamp_ns,
                    authorized_at_ns=self.authorized_at_ns,
                    motion_release=self.state == 'authorized')
