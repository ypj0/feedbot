class Convergence:
    def __init__(self):
        self.phase = 'idle'
        self.reason = 'explicit_start_required'
        self.requests = 0
        self.confirmed = 0
        self.map_key = None
        self.source_key = None
        self.pending = False
        self.acknowledged = False
        self.last_stamp = 0
        self.request_scan_stamp = None
        self.request_posterior_stamp = None
        self.sent = None
        self.operation = None
        self.budget_s = None

    def start(self, now, view, operation='refresh'):
        if self.phase == 'running':
            raise ValueError('convergence already running')
        if operation not in ('refresh', 'acquire'):
            raise ValueError('unknown convergence operation')
        if view['error']:
            raise ValueError(view['error'])
        result = view['result'] or {}
        compatible = (result.get('accepted') or
                      (result.get('pending_convergence') and
                       result.get('innovation', {}).get('compatible')))
        if (not view['map_key'][1] or not compatible or
                view['phase'] not in ('waiting_pair', 'pending_convergence', 'verified_pair')):
            raise ValueError('no compatible posterior on current map')
        self.phase = 'running'
        self.reason = 'refreshing_posterior'
        self.requests = 0
        self.confirmed = 0
        self.map_key = view['map_key']
        self.source_key = view['source_key']
        self.counters = view['counters']
        self.operation = operation
        self.budget_s = 30. if operation == 'acquire' else 10.
        self.deadline = now + self.budget_s
        self.next_request = now
        self.pending = False
        self.acknowledged = False
        self.last_stamp = view['observation_stamp_ns']
        self.request_scan_stamp = None
        self.request_posterior_stamp = None
        self.sent = None

    def fail(self, reason):
        if self.phase == 'running':
            self.phase, self.reason = 'failed', reason

    def response(self, successful):
        if self.phase != 'running' or not self.pending:
            return
        if not successful:
            self.fail('nomotion_service_failed')
        else:
            self.acknowledged = True

    def step(self, now, view):
        if self.phase != 'running':
            return False
        if view['error']:
            self.fail(view['error'])
        elif view['map_key'] != self.map_key:
            self.fail('map_changed')
        elif view['source_key'] != self.source_key:
            self.fail('posterior_source_changed')
        elif view['counters'] != self.counters:
            self.fail('chassis_fault_or_command_counter_changed')
        elif view['phase'] in ('error', 'input_stale', 'waiting_map', 'rejected'):
            self.fail('verification_' + view['phase'])
        elif now >= self.deadline:
            self.fail('budget_exhausted')
        if self.phase != 'running':
            return False
        if self.pending:
            new_pair = (view['observation_stamp_ns'] > self.request_posterior_stamp and
                        view['phase'] in ('verified_pair', 'pending_convergence'))
            if self.acknowledged and new_pair:
                self.last_stamp = view['observation_stamp_ns']
                self.confirmed += 1
                self.pending = False
                if view['phase'] == 'verified_pair':
                    self.phase, self.reason = 'succeeded', 'new_pair_verified'
                elif self.operation == 'refresh' and self.requests >= 10:
                    self.fail('budget_exhausted')
            elif now - self.sent >= 1.:
                self.fail('new_pair_timeout' if self.acknowledged else 'service_response_timeout')
        if self.phase != 'running' or self.pending or now < self.next_request:
            return False
        if view['scan_stamp_ns'] <= self.last_stamp:
            return False
        self.pending = True
        self.acknowledged = False
        self.request_scan_stamp = view['scan_stamp_ns']
        self.request_posterior_stamp = max(self.last_stamp, view['observation_stamp_ns'])
        self.sent = now
        self.next_request = now + 1.01
        self.requests += 1
        return True

    def status(self):
        return dict(phase=self.phase, reason=self.reason, requests=self.requests,
                    operation=self.operation, budget_s=self.budget_s,
                    confirmed=self.confirmed, map_key=self.map_key,
                    source_key=self.source_key,
                    request_scan_stamp_ns=self.request_scan_stamp,
                    request_posterior_stamp_ns=self.request_posterior_stamp,
                    request_sent_monotonic_s=self.sent,
                    request_pending=self.pending, request_acknowledged=self.acknowledged,
                    observation_stamp_ns=self.last_stamp, localization_accepted=False)
