#!/usr/bin/env python3
"""Offline round-trip check for dock_icp (no ROS). Run with numpy available."""

import math
import sys

import numpy as np

from dock_icp import icp_align, refine_pose


def _rot(yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.array([[c, -s], [s, c]], dtype=float)


def main():
    # 非对称桩模板：前方平板 + 左右长短不一的墙段
    template = []
    for y in np.linspace(-0.25, 0.25, 31):
        template.append((0.5, y))
    for x in np.linspace(0.30, 0.60, 13):
        template.append((x, 0.30))
    for x in np.linspace(0.35, 0.55, 9):
        template.append((x, -0.40))
    template = np.asarray(template, dtype=float)

    cases = [
        (0.0, 0.0, 0.0),
        (0.04, -0.03, math.radians(4.0)),
        (-0.05, 0.02, math.radians(-7.0)),
        (0.10, 0.0, math.radians(15.0)),
    ]
    rng = np.random.default_rng(7)
    for dx, dy, dyaw in cases:
        r = _rot(-dyaw)
        current = (template - np.array([dx, dy])) @ r.T
        current = current + rng.normal(0.0, 0.005, current.shape)
        theta, tx, ty, rms, inliers = icp_align(template, current)
        pose = refine_pose([0.0, 0.0, 0.0], theta, tx, ty)
        err = math.hypot(pose[0] - dx, pose[1] - dy)
        yerr = abs((pose[2] - dyaw + math.pi) % (2 * math.pi) - math.pi)
        print('case dx=%+.3f dy=%+.3f dyaw=%+.2fdeg -> pose=%s rms=%.5f inliers=%.2f err=%.5f yerr=%.5f'
              % (dx, dy, math.degrees(dyaw), [round(v, 5) for v in pose],
                 rms, inliers, err, yerr))
        assert err < 0.03, 'position error exceeds docking tolerance'
        assert yerr < 0.035, 'yaw error exceeds docking tolerance'
        assert rms < 0.05 and inliers > 0.5, 'lock thresholds not met'
    print('dock_icp round-trip OK')


if __name__ == '__main__':
    sys.exit(main())
