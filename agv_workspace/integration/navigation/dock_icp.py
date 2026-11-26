#!/usr/bin/env python3
"""Numpy-only 2D point-to-point ICP and scan helpers for dock perception.

Conventions:
- Points are Nx2 arrays in the lidar scan frame; +x is forward, +y is left.
- ``icp_align(src, dst)`` returns ``(theta, tx, ty, rms, inlier_ratio)`` such
  that ``dst ~= R(theta) @ src + t``. Pass ``src=template``, ``dst=current``
  to recover the transform from the calibration scan frame to the current
  scan frame.
- ``refine_pose`` converts that residual plus the calibration-time map pose
  into a refined current map pose.
"""

import math

import numpy as np


def scan_front_points(msg, angle_min=-1.0, angle_max=1.0, range_min=0.15,
                      range_max=2.0, step=4):
    """Crop and downsample a sensor_msgs/LaserScan to an Nx2 numpy array."""
    pts = []
    n = len(msg.ranges)
    for i in range(0, n, step):
        ang = msg.angle_min + i * msg.angle_increment
        if ang < angle_min or ang > angle_max:
            continue
        r = msg.ranges[i]
        if math.isfinite(r) and range_min <= r <= range_max:
            pts.append((r * math.cos(ang), r * math.sin(ang)))
    return np.asarray(pts, dtype=float).reshape(-1, 2)


def _rotation(theta):
    c, s = math.cos(theta), math.sin(theta)
    return np.array([[c, -s], [s, c]], dtype=float)


def icp_align(src, dst, max_iter=40, tol=1e-4, outlier_dist=0.06):
    """Point-to-point ICP.

    Returns ``(theta, tx, ty, rms, inlier_ratio)`` with ``dst ~= R @ src + t``.
    ``rms`` is the mean nearest-neighbour distance after alignment and
    ``inlier_ratio`` the fraction of correspondences within ``outlier_dist``.
    """
    src = np.asarray(src, dtype=float).reshape(-1, 2)
    dst = np.asarray(dst, dtype=float).reshape(-1, 2)
    if src.shape[0] < 4 or dst.shape[0] < 4:
        return 0.0, 0.0, 0.0, float('inf'), 0.0

    # 粗角度搜索：线状桩模板在零初始下容易陷入局部最优。
    best_yaw = 0.0
    best_score = float('inf')
    for deg in range(-30, 31, 3):
        yaw = math.radians(deg)
        cand = src @ _rotation(yaw).T
        d2 = ((cand[:, None, :] - dst[None, :, :]) ** 2).sum(axis=2)
        dist = np.sqrt(d2.min(axis=1))
        keep = max(4, int(dist.size * 0.8))
        score = float(np.sort(dist)[:keep].mean())
        if score < best_score:
            best_score = score
            best_yaw = yaw

    total_rot = _rotation(best_yaw)
    total_trans = np.zeros(2)
    work = src @ total_rot.T
    for it in range(max_iter):
        d2 = ((work[:, None, :] - dst[None, :, :]) ** 2).sum(axis=2)
        idx = d2.argmin(axis=1)
        dist = np.sqrt(d2[np.arange(work.shape[0]), idx])
        threshold = outlier_dist * 3.0 if it == 0 else outlier_dist
        keep = dist <= threshold
        if keep.sum() < 4:
            break
        s = work[keep]
        m = dst[idx[keep]]
        sc = s - s.mean(axis=0)
        mc = m - m.mean(axis=0)
        h = sc.T @ mc
        u, _, vt = np.linalg.svd(h)
        inc_rot = vt.T @ u.T
        if np.linalg.det(inc_rot) < 0:
            vt[-1, :] *= -1
            inc_rot = vt.T @ u.T
        inc_trans = m.mean(axis=0) - inc_rot @ s.mean(axis=0)
        new_work = work @ inc_rot.T + inc_trans
        delta = np.abs(new_work - work).max()
        work = new_work
        total_trans = inc_rot @ total_trans + inc_trans
        total_rot = inc_rot @ total_rot
        if delta < tol:
            break

    d2 = ((work[:, None, :] - dst[None, :, :]) ** 2).sum(axis=2)
    dist = np.sqrt(d2.min(axis=1))
    rms = float(dist.mean()) if dist.size else float('inf')
    inlier = float((dist <= outlier_dist).mean()) if dist.size else 0.0
    theta = math.atan2(total_rot[1, 0], total_rot[0, 0])
    return theta, float(total_trans[0]), float(total_trans[1]), rms, inlier


def icp_residual_to_relative(theta, tx, ty):
    """Convert template->current ``(R, t)`` to current pose relative to template."""
    rot = _rotation(theta)
    rt = rot.T @ np.array([tx, ty])
    return (-rt[0], -rt[1], -theta)


def compose_pose(base, rel):
    """Compose map pose ``base=[x,y,yaw]`` with relative pose ``rel=[x,y,yaw]``."""
    bx, by, byaw = base
    rx, ry, ryaw = rel
    c, s = math.cos(byaw), math.sin(byaw)
    x = bx + c * rx - s * ry
    y = by + s * rx + c * ry
    yaw = byaw + ryaw
    yaw = math.fmod(math.fmod(yaw, 2 * math.pi) + 2 * math.pi, 2 * math.pi)
    if yaw > math.pi:
        yaw -= 2 * math.pi
    return [x, y, yaw]


def refine_pose(template_pose, theta, tx, ty):
    """Refine current map pose from the calibration pose and an ICP residual."""
    rel = icp_residual_to_relative(theta, tx, ty)
    return compose_pose(template_pose, rel)
