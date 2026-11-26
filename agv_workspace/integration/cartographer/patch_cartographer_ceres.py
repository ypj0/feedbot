#!/usr/bin/env python3
"""Patch Cartographer 1.x to build against Ceres 2.2 (Ubuntu 24.04/Jazzy).

Usage:
    patch_cartographer_ceres.py /path/to/cartographer-source

The upstream 1.0.0 source uses the Ceres LocalParameterization API removed in
Ceres 2.2.0, and GCC 13 flags Eigen 3.4's SVD code as maybe-uninitialized. This
script applies the same Manifold conversion and relaxes only that compiler
false positive.
"""

import re
import sys
from pathlib import Path


def patch_rotation_parameterization(source: Path) -> None:
    path = source / "cartographer/mapping/internal/3d/rotation_parameterization.h"
    path.write_text(
        """/*
 * Copyright 2016 The Cartographer Authors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef CARTOGRAPHER_MAPPING_INTERNAL_3D_ROTATION_PARAMETERIZATION_H_
#define CARTOGRAPHER_MAPPING_INTERNAL_3D_ROTATION_PARAMETERIZATION_H_

#include "cartographer/common/math.h"
#include "ceres/autodiff_manifold.h"
#include "ceres/jet.h"
#include "ceres/manifold.h"
#include "ceres/rotation.h"

namespace cartographer {
namespace mapping {

struct YawOnlyQuaternionOperations {
  template <typename T>
  bool Plus(const T* x, const T* delta, T* x_plus_delta) const {
    const T clamped_delta = common::Clamp(delta[0], T(-0.5), T(0.5));
    T q_delta[4];
    q_delta[0] = ceres::sqrt(1. - clamped_delta * clamped_delta);
    q_delta[1] = T(0.);
    q_delta[2] = T(0.);
    q_delta[3] = clamped_delta;
    ceres::QuaternionProduct(q_delta, x, x_plus_delta);
    return true;
  }
  template <typename T>
  bool Minus(const T* y, const T* x, T* y_minus_x) const {
    T minus_x[4] = {x[0], -x[1], -x[2], -x[3]};
    T q_delta[4];
    ceres::QuaternionProduct(y, minus_x, q_delta);
    y_minus_x[0] = q_delta[3];
    return true;
  }
};

struct ConstantYawQuaternionOperations {
  template <typename T>
  bool Plus(const T* x, const T* delta, T* x_plus_delta) const {
    const T delta_norm =
        ceres::sqrt(common::Pow2(delta[0]) + common::Pow2(delta[1]));
    const T sin_delta_over_delta =
        delta_norm < 1e-6 ? T(1.) : ceres::sin(delta_norm) / delta_norm;
    T q_delta[4];
    q_delta[0] = delta_norm < 1e-6 ? T(1.) : ceres::cos(delta_norm);
    q_delta[1] = sin_delta_over_delta * delta[0];
    q_delta[2] = sin_delta_over_delta * delta[1];
    q_delta[3] = T(0.);
    ceres::QuaternionProduct(x, q_delta, x_plus_delta);
    return true;
  }
  template <typename T>
  bool Minus(const T* y, const T* x, T* y_minus_x) const {
    T minus_x[4] = {x[0], -x[1], -x[2], -x[3]};
    T q_delta[4];
    ceres::QuaternionProduct(minus_x, y, q_delta);
    const T& cos_delta_norm = q_delta[0];
    const T sin_delta_norm =
        ceres::sqrt(common::Pow2(q_delta[1]) + common::Pow2(q_delta[2]));
    const T delta_norm = atan2(sin_delta_norm, cos_delta_norm);
    const T delta_over_sin_delta =
        delta_norm < 1e-6 ? T(1.) : delta_norm / sin_delta_norm;
    y_minus_x[0] = q_delta[1] * delta_over_sin_delta;
    y_minus_x[1] = q_delta[2] * delta_over_sin_delta;
    return true;
  }
};

using YawOnlyQuaternionManifold =
    ceres::AutoDiffManifold<YawOnlyQuaternionOperations, 4, 1>;
using ConstantYawQuaternionManifold =
    ceres::AutoDiffManifold<ConstantYawQuaternionOperations, 4, 2>;

}  // namespace mapping
}  // namespace cartographer

#endif  // CARTOGRAPHER_MAPPING_INTERNAL_3D_ROTATION_PARAMETERIZATION_H_
"""
    )


def rewrite(source: Path, relative_path: str) -> None:
    path = source / relative_path
    text = path.read_text()
    text = re.sub(
        r"ceres::AutoDiffManifold<\s*YawOnlyQuaternionPlus\s*,\s*4\s*,\s*1\s*>",
        "YawOnlyQuaternionManifold",
        text,
    )
    text = re.sub(
        r"ceres::AutoDiffManifold<\s*ConstantYawQuaternionPlus\s*,\s*4\s*,\s*2\s*>",
        "ConstantYawQuaternionManifold",
        text,
    )
    text = re.sub(
        r"ceres::AutoDiffLocalParameterization<\s*YawOnlyQuaternionPlus\s*,\s*4\s*,\s*1\s*>",
        "YawOnlyQuaternionManifold",
        text,
    )
    text = re.sub(
        r"ceres::AutoDiffLocalParameterization<\s*ConstantYawQuaternionPlus\s*,\s*4\s*,\s*2\s*>",
        "ConstantYawQuaternionManifold",
        text,
    )
    text = text.replace("ceres::SubsetParameterization", "ceres::SubsetManifold")
    text = text.replace("translation_parameterization", "translation_manifold")
    path.write_text(text)


def add_include(source: Path, relative_path: str) -> None:
    path = source / relative_path
    text = path.read_text()
    if '#include "ceres/manifold.h"' not in text:
        text = text.replace(
            '#include "ceres/ceres.h"',
            '#include "ceres/ceres.h"\n#include "ceres/manifold.h"',
        )
        path.write_text(text)


def patch_compiler_flags(source: Path) -> None:
    path = source / "cmake/functions.cmake"
    text = path.read_text()
    text = text.replace(
        'google_add_flag(GOOG_CXX_FLAGS "-Werror=uninitialized")',
        'google_add_flag(GOOG_CXX_FLAGS "-Werror=uninitialized")\n'
        '    google_add_flag(GOOG_CXX_FLAGS "-Wno-maybe-uninitialized")',
    )
    path.write_text(text)


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__, file=sys.stderr)
        return 2
    source = Path(sys.argv[1])
    if not source.is_dir():
        print(f"not a directory: {source}", file=sys.stderr)
        return 2

    patch_rotation_parameterization(source)
    for relative in (
        "cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.cc",
        "cartographer/mapping/internal/imu_based_pose_extrapolator.cc",
        "cartographer/mapping/internal/optimization/optimization_problem_3d.cc",
        "cartographer/mapping/internal/optimization/optimization_problem_2d.cc",
    ):
        rewrite(source, relative)
    for relative in (
        "cartographer/mapping/internal/3d/scan_matching/ceres_scan_matcher_3d.cc",
        "cartographer/mapping/internal/imu_based_pose_extrapolator.cc",
        "cartographer/mapping/internal/optimization/ceres_pose.h",
    ):
        add_include(source, relative)
    patch_compiler_flags(source)
    print("patched cartographer for Ceres 2.2")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
