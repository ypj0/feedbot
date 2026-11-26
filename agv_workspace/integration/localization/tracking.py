class Tracking:
    def __init__(self):
        self.identity = None
        self.fault_counters = None
        self.required_after_ns = 0

    def invalidate(self, now_ns):
        if self.identity is not None:
            self.required_after_ns = max(self.required_after_ns, now_ns)

    def update(self, now_ns, view, acquiring=False):
        output = dict(phase='hold', reason=None, localization_ready=False,
                      motion_authorized=False, stamp_ns=now_ns,
                      map_key=view.get('map_key'), source_key=view.get('source_key'),
                      observation_stamp_ns=view.get('observation_stamp_ns'))
        if view['error']:
            self.invalidate(now_ns)
            return dict(output, phase='invalid', reason=view['error'])
        identity = (view['map_key'], view['source_key'])
        fault_counters = view['counters'][1:]
        changed = self.identity is not None and identity != self.identity
        faulted = self.fault_counters is not None and fault_counters != self.fault_counters
        self.identity = identity
        self.fault_counters = fault_counters
        if changed or faulted:
            self.required_after_ns = max(self.required_after_ns, now_ns)
            return dict(output, phase='invalid', reason='map_or_source_changed' if changed else
                        'chassis_fault_counter_changed')
        if view['phase'] in ('error', 'input_stale', 'waiting_map', 'rejected'):
            self.required_after_ns = max(self.required_after_ns, now_ns)
            return dict(output, phase='invalid', reason='verification_' + view['phase'])
        if acquiring:
            return dict(output, reason='acquisition_running')
        if view['phase'] != 'verified_pair':
            return dict(output, reason='verification_' + view['phase'])
        stamp_ns = view['observation_stamp_ns']
        if not 0 <= now_ns - stamp_ns < 2000000000:
            return dict(output, reason='posterior_source_time_invalid')
        if stamp_ns <= self.required_after_ns:
            return dict(output, reason='post_invalidation_pair_required')
        if not (view['result'] or {}).get('accepted'):
            self.invalidate(now_ns)
            return dict(output, phase='invalid', reason='inconsistent_verified_result')
        return dict(output, phase='tracking', reason='current_pair_verified', localization_ready=True)
