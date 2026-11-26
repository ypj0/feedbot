#!/usr/bin/env python3
"""纯函数：用当前激光帧与地图障碍距离场做暴力全局扫描匹配。

从旧 scan_relocalize.py 中抽取，供 Cartographer 重定位复用，不依赖 ROS。
"""

import math
from collections import deque

import numpy as np


def build_distance_field(map_msg):
    """把 OccupancyGrid 的 100 障碍栅格建成 Manhattan 距离场，返回 (w, h, dist)。"""
    w, h = map_msg.info.width, map_msg.info.height
    arr = np.array(map_msg.data, dtype=np.int8).reshape(h, w)
    occ = arr == 100
    dist = np.full((h, w), 1e9, dtype=np.float32)
    q = deque()
    ys, xs = np.where(occ)
    for y, x in zip(ys.tolist(), xs.tolist()):
        dist[y, x] = 0.0
        q.append((y, x))
    while q:
        y, x = q.popleft()
        d0 = dist[y, x] + 1
        for ny, nx in ((y - 1, x), (y + 1, x), (y, x - 1), (y, x + 1)):
            if 0 <= ny < h and 0 <= nx < w and dist[ny, nx] > d0:
                dist[ny, nx] = d0
                q.append((ny, nx))
    return w, h, dist


def laser_points(scan, laser_max_range, base_link_laser_x=0.27):
    """有效激光点变换到 base_link 坐标（激光相对车体的纵向偏移默认 0.27m）。"""
    bx, by = [], []
    for i, r in enumerate(scan.ranges):
        if math.isfinite(r) and 0.05 < r < laser_max_range:
            a = scan.angle_min + i * scan.angle_increment
            bx.append(base_link_laser_x + r * math.cos(a))
            by.append(r * math.sin(a))
    return np.array(bx, dtype=np.float32), np.array(by, dtype=np.float32)


def score(px, py, pyaw, w, h, dist, bx, by, res, ox, oy):
    c, s = math.cos(pyaw), math.sin(pyaw)
    gx = ((px + bx * c - by * s - ox) / res).astype(np.int32)
    gy = ((py + bx * s + by * c - oy) / res).astype(np.int32)
    valid = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
    if valid.sum() < len(bx) * 0.5:
        return 1e9
    return float(dist[gy[valid], gx[valid]].mean())


def search(map_msg, scan, laser_max_range=3.5, min_points=50):
    """粗网格暴力搜索 + 前三候选细化，返回 (score, x, y, yaw) 或 None。"""
    res = map_msg.info.resolution
    ox = map_msg.info.origin.position.x
    oy = map_msg.info.origin.position.y
    w, h, dist = build_distance_field(map_msg)
    bx, by = laser_points(scan, laser_max_range)
    if len(bx) < min_points:
        raise RuntimeError('not enough laser points')

    coarse = []
    for deg in range(-180, 180, 15):
        pyaw = math.radians(deg)
        c, s = math.cos(pyaw), math.sin(pyaw)
        rx = bx * c - by * s
        ry = bx * s + by * c
        best = (1e9, 0.0, 0.0)
        for py in np.arange(oy, oy + h * res, 0.10):
            gy = ((py + ry - oy) / res).astype(np.int32)
            for px in np.arange(ox, ox + w * res, 0.10):
                gx = ((px + rx - ox) / res).astype(np.int32)
                valid = (gx >= 0) & (gx < w) & (gy >= 0) & (gy < h)
                if valid.sum() < len(bx) * 0.5:
                    continue
                sc = float(dist[gy[valid], gx[valid]].mean())
                if sc < best[0]:
                    best = (sc, px, py)
        coarse.append((best[0], best[1], best[2], pyaw))
    coarse.sort(key=lambda z: z[0])

    best = None
    for sc, px, py, pyaw in coarse[:3]:
        for x in np.arange(px - 0.15, px + 0.16, 0.02):
            for y in np.arange(py - 0.15, py + 0.16, 0.02):
                for deg in np.arange(math.degrees(pyaw) - 12, math.degrees(pyaw) + 13, 2):
                    sc2 = score(x, y, math.radians(deg), w, h, dist, bx, by, res, ox, oy)
                    if best is None or sc2 < best[0]:
                        best = (sc2, x, y, math.radians(deg))
    return best
