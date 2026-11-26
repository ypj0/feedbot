import hashlib
import json
import math

from global_localization import search
from localization_observation import Observations


class Verification:
    def __init__(self):
        self.grid = None
        self.map_digest = None
        self.generation = 0
        self.posterior_source = None
        self.posterior_generation = 0
        self.observations = Observations()
        self.last_observation_ns = 0
        self.latest_scan_ns = 0
        self.result = None
        self.result_stamp_ns = 0
        self.result_posterior = None
        self.errors = {}

    def set_posterior_source(self, source, now_ns):
        if source != self.posterior_source:
            if self.posterior_generation:
                self.observations.clear(now_ns)
                self.last_observation_ns = 0
                self.result = None
                self.result_stamp_ns = 0
                self.result_posterior = None
            self.posterior_generation += 1
            self.posterior_source = source
        if source is None:
            self.errors['posterior_source'] = 'AMCL posterior requires exactly one publisher'
        else:
            self.errors.pop('posterior_source', None)

    def set_map(self, grid, now_ns):
        digest = hashlib.sha256(json.dumps(grid, sort_keys=True, separators=(',', ':')).encode()).hexdigest()
        self.errors.pop('map', None)
        if digest == self.map_digest:
            return
        if self.grid is not None:
            self.observations.clear(now_ns)
            self.latest_scan_ns = 0
        self.grid = grid
        self.map_digest = digest
        self.generation += 1
        self.last_observation_ns = 0
        self.result = None
        self.result_stamp_ns = 0
        self.result_posterior = None

    def scan(self, stamp_ns, points):
        self.observations.scan(stamp_ns, points)
        self.latest_scan_ns = max(self.latest_scan_ns, stamp_ns)
        self.errors.pop('scan', None)

    def posterior(self, stamp_ns, frame, pose, covariance):
        if frame != 'map' or len(covariance) != 36 or not all(
                math.isfinite(value) for value in pose + list(covariance)):
            raise ValueError('invalid map-frame AMCL posterior')
        self.observations.posterior(stamp_ns, frame, pose, covariance)
        self.errors.pop('posterior', None)

    def prepare(self, now_ns):
        if self.grid is None or self.posterior_source is None:
            return None
        paired = self.observations.latest(now_ns, self.last_observation_ns)
        if paired is None:
            return None
        stamp_ns, points, (pose, covariance) = paired
        self.last_observation_ns = stamp_ns
        return dict(generation=self.generation, posterior_generation=self.posterior_generation,
                    map_digest=self.map_digest,
                    stamp_ns=stamp_ns, grid=self.grid, points=points, pose=pose,
                    posterior=dict(stamp_ns=stamp_ns, frame_id='map', pose=list(pose),
                                   covariance=list(covariance)),
                    covariance=[[covariance[row * 6 + column] for column in (0, 1, 5)]
                                for row in (0, 1, 5)])

    def complete(self, job, result):
        if (job['generation'] != self.generation or
                job['posterior_generation'] != self.posterior_generation):
            return False
        self.result = result
        self.result_stamp_ns = job['stamp_ns']
        self.result_posterior = job['posterior']
        self.errors.pop('evaluation', None)
        return True

    def snapshot(self, now_ns):
        output = dict(map_generation=self.generation, map_digest=self.map_digest,
                      posterior_generation=self.posterior_generation,
                      posterior_source=self.posterior_source,
                      observation_stamp_ns=self.result_stamp_ns, result=self.result,
                      posterior=self.result_posterior,
                      pair_verified=False, localization_accepted=False)
        age = (now_ns - self.result_stamp_ns) / 1e9 if self.result_stamp_ns else None
        output['observation_age_s'] = age
        if self.errors:
            phase, reason = 'error', json.dumps(self.errors, ensure_ascii=False)
        elif self.grid is None:
            phase, reason = 'waiting_map', '等待当前地图'
        elif not self.latest_scan_ns or not 0 <= now_ns - self.latest_scan_ns < 500000000:
            phase, reason = 'input_stale', '无新鲜扫描，不判定定位有效'
        elif age is None or not 0 <= age < 2:
            phase, reason = 'waiting_pair', '等待新鲜同时间扫描与AMCL后验；静止等待不等于丢定位'
        elif self.result.get('accepted'):
            phase, reason = 'verified_pair', '同帧后验与激光一致；不代表导航授权或运动验收'
            output['pair_verified'] = True
        elif self.result.get('pending_convergence'):
            phase, reason = 'pending_convergence', self.result['reason']
        else:
            phase, reason = 'rejected', self.result['reason']
        return dict(output, phase=phase, reason=reason)


def evaluate(job):
    return search(job['grid'], job['points'], job['pose'], job['covariance'])
