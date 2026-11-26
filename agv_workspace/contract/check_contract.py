#!/usr/bin/env python3
"""静态核对契约：feedbot_contract.yaml 与其余配置、msg 枚举、C++ 枚举是否一致。

用法：check_contract.py [agv_workspace 根目录]   （默认取本文件上一级）
有任何不一致则逐条打印并以非零退出。只读，不修改任何文件。
"""
import math
import pathlib
import re
import sys
import xml.etree.ElementTree as ET

import yaml

CONTRACT = "contract/feedbot_contract.yaml"
CONTROLLERS = "drivers/chassis_ros2/config/controllers.yaml"
URDF = "drivers/chassis_ros2/config/chassis.urdf"
SESSION_HPP = "drivers/chassis_ros2/include/feedbot_chassis_ros2/session.hpp"
EKF = "integration/ekf/config/ekf.yaml"
AUTHORITY_HPP = "motion_authority/include/feedbot_motion_authority/authority.hpp"
MSG_REQUEST = "motion_interfaces/msg/MotionRequest.msg"
MSG_STATUS = "motion_interfaces/msg/MotionAuthorityStatus.msg"

TOL = 1e-9


def load_yaml(path):
    with open(path, encoding="utf-8") as handle:
        return yaml.safe_load(handle)


def params(document, node_key):
    return document[node_key]["ros__parameters"]


def close(a, b):
    return math.isclose(float(a), float(b), rel_tol=0.0, abs_tol=TOL)


def snake(name):
    return re.sub(r"(?<!^)(?=[A-Z])", "_", name).upper()


def cxx_enum(text, enum_name):
    """解析 `enum class X : T { A = 0, B, C }`，返回 {名字: 值}。"""
    match = re.search(r"enum\s+class\s+%s\b[^{]*\{(.*?)\}" % re.escape(enum_name), text, re.S)
    if not match:
        raise ValueError("enum %s not found" % enum_name)
    body = re.sub(r"//[^\n]*", "", match.group(1))
    values, nxt = {}, 0
    for item in body.split(","):
        item = item.strip()
        if not item:
            continue
        if "=" in item:
            name, value = [part.strip() for part in item.split("=")]
            nxt = int(value, 0)
        else:
            name = item
        values[name] = nxt
        nxt += 1
    return values


def msg_constants(text, prefix):
    found = {}
    for line in text.splitlines():
        match = re.match(r"\s*uint8\s+(%s_[A-Z0-9_]+)\s*=\s*(\d+)" % prefix, line)
        if match:
            found[match.group(1)] = int(match.group(2))
    return found


def check_enum(errors, label, cxx, msg, prefix):
    expected = {"%s_%s" % (prefix, snake(name)): value for name, value in cxx.items()}
    for name, value in expected.items():
        if name not in msg:
            errors.append("%s: msg 缺少常量 %s=%d" % (label, name, value))
        elif msg[name] != value:
            errors.append("%s: %s msg=%d 但 authority.hpp=%d" % (label, name, msg[name], value))
    for name in msg:
        if name not in expected:
            errors.append("%s: msg 多出 authority.hpp 中不存在的常量 %s" % (label, name))


def check(root):
    root = pathlib.Path(root)
    errors = []

    contract = params(load_yaml(root / CONTRACT), "/**")["feedbot_contract"]
    timing, limits = contract["timing"], contract["limits"]
    topics, frames = contract["topics"], contract["frames"]
    nominal = contract["chassis_nominal"]
    rate = float(timing["control_rate_hz"])
    period_ms = 1000.0 / rate

    # --- 契约自身的内部约束 ---
    for key in ("control_rate_hz", "lease_timeout_ms", "max_stamp_age_ms", "max_stamp_lead_ms",
                "downstream_cmd_vel_timeout_ms", "status_rate_hz", "stats_period_s", "control_step_p99_budget_ms"):
        if not (isinstance(timing[key], (int, float)) and timing[key] > 0):
            errors.append("contract: timing.%s 必须为正数" % key)
    for key in ("max_linear_mps", "max_angular_rps", "request_queue_depth"):
        if not (isinstance(limits[key], (int, float)) and limits[key] > 0):
            errors.append("contract: limits.%s 必须为正数" % key)
    if not isinstance(limits["request_queue_depth"], int):
        errors.append("contract: limits.request_queue_depth 必须为整数")
    if timing["lease_timeout_ms"] < 2 * period_ms:
        errors.append("contract: lease_timeout_ms(%s) 小于 2 个控制周期(%.1f ms)"
                      % (timing["lease_timeout_ms"], 2 * period_ms))
    if timing["status_rate_hz"] > 0 and (rate % timing["status_rate_hz"] != 0):
        errors.append("contract: status_rate_hz(%s) 必须整除 control_rate_hz(%s)"
                      % (timing["status_rate_hz"], rate))
    if timing["downstream_cmd_vel_timeout_ms"] < 2 * period_ms:
        errors.append("contract: downstream_cmd_vel_timeout_ms 小于 2 个控制周期")
    if timing["control_step_p99_budget_ms"] >= period_ms:
        errors.append("contract: control_step_p99_budget_ms(%s) 必须小于控制周期(%.1f ms)"
                      % (timing["control_step_p99_budget_ms"], period_ms))
    if timing["max_stamp_age_ms"] >= timing["lease_timeout_ms"]:
        errors.append("contract: max_stamp_age_ms 应小于 lease_timeout_ms")
    values = [v for group in ("topics",) for v in contract[group].values()]
    if len(set(values)) != len(values):
        errors.append("contract: topics 中有重复的名字")

    # --- controllers.yaml （可选，构建时不一定有）---
    controllers_path = root / CONTROLLERS
    if controllers_path.exists():
        controllers = load_yaml(controllers_path)
        manager = params(controllers, "controller_manager")
        wheel = params(controllers, "wheel")
    if controllers_path.exists():
        if not close(manager["update_rate"], rate):
            errors.append("controllers.yaml: update_rate=%s 与契约 control_rate_hz=%s 不一致"
                          % (manager["update_rate"], rate))
        if not close(wheel["publish_rate"], rate):
            errors.append("controllers.yaml: wheel.publish_rate=%s 与契约不一致" % wheel["publish_rate"])
        if not close(float(wheel["cmd_vel_timeout"]) * 1000.0, timing["downstream_cmd_vel_timeout_ms"]):
            errors.append("controllers.yaml: cmd_vel_timeout=%s s 与契约 downstream_cmd_vel_timeout_ms=%s 不一致"
                          % (wheel["cmd_vel_timeout"], timing["downstream_cmd_vel_timeout_ms"]))
        if not close(wheel["wheel_radius"], nominal["wheel_radius_m"]):
            errors.append("controllers.yaml: wheel_radius=%s 与契约 %s 不一致"
                          % (wheel["wheel_radius"], nominal["wheel_radius_m"]))
        if not close(wheel["wheel_separation"], nominal["wheel_separation_m"]):
            errors.append("controllers.yaml: wheel_separation=%s 与契约 %s 不一致"
                          % (wheel["wheel_separation"], nominal["wheel_separation_m"]))
        if wheel.get("odom_frame_id") != frames["odom"] or wheel.get("base_frame_id") != frames["base"]:
            errors.append("controllers.yaml: odom/base frame 与契约不一致")
        if wheel.get("enable_odom_tf") is not False:
            errors.append("controllers.yaml: enable_odom_tf 必须为 false（odom→base_link 只由 EKF 发布）")
        if wheel.get("open_loop") is not False or wheel.get("position_feedback") is not True:
            errors.append("controllers.yaml: 须 open_loop=false 且 position_feedback=true")

    # --- ekf.yaml （可选）---
    ekf_path = root / EKF
    if ekf_path.exists():
        ekf = params(load_yaml(ekf_path), "ekf_filter_node")
    if ekf_path.exists():
        if not close(ekf["frequency"], rate):
            errors.append("ekf.yaml: frequency=%s 与契约 control_rate_hz=%s 不一致" % (ekf["frequency"], rate))
        if ekf.get("odom0") != topics["wheel_odom"]:
            errors.append("ekf.yaml: odom0=%s 与契约 wheel_odom=%s 不一致" % (ekf.get("odom0"), topics["wheel_odom"]))
        if ekf.get("imu0") != topics["imu"]:
            errors.append("ekf.yaml: imu0=%s 与契约 imu=%s 不一致" % (ekf.get("imu0"), topics["imu"]))
        if ekf.get("odom_frame") != frames["odom"] or ekf.get("base_link_frame") != frames["base"]:
            errors.append("ekf.yaml: odom/base_link frame 与契约不一致")
        if ekf.get("two_d_mode") is not True or ekf.get("publish_tf") is not True:
            errors.append("ekf.yaml: 须 two_d_mode=true 且 publish_tf=true")

    # --- URDF 轮子几何（可选）---
    urdf_path = root / URDF
    if urdf_path.exists():
        joints = {}
        for joint in ET.parse(urdf_path).getroot().iter("joint"):
            origin = joint.find("origin")
            if origin is not None:
                joints[joint.get("name")] = [float(v) for v in origin.get("xyz").split()]
        half = nominal["wheel_separation_m"] / 2.0
        for name, sign in (("left_wheel_joint", 1.0), ("right_wheel_joint", -1.0)):
            xyz = joints.get(name)
            if xyz is None:
                errors.append("chassis.urdf: 缺少 %s" % name)
                continue
            if not close(xyz[1], sign * half):
                errors.append("chassis.urdf: %s y=%s 与契约轮距/2=%s 不一致" % (name, xyz[1], sign * half))
            if not close(xyz[2], nominal["wheel_radius_m"]):
                errors.append("chassis.urdf: %s z=%s 与契约轮径 %s 不一致" % (name, xyz[2], nominal["wheel_radius_m"]))

    # --- session.hpp（可选）---
    session_path = root / SESSION_HPP
    if session_path.exists():
        header = session_path.read_text(encoding="utf-8")
        for key, contract_key in (("radius", "wheel_radius_m"), ("separation", "wheel_separation_m")):
            match = re.search(r"static\s+constexpr\s+double\s+%s\s*=\s*([0-9.eE+-]+)\s*;" % key, header)
            if not match:
                errors.append("session.hpp: 找不到 Wheels::%s" % key)
            elif not close(match.group(1), nominal[contract_key]):
                errors.append("session.hpp: %s=%s 与契约 %s 不一致" % (key, match.group(1), nominal[contract_key]))

    # --- C++ 枚举 ↔ msg 常量 ---
    authority = (root / AUTHORITY_HPP).read_text(encoding="utf-8")
    request_msg = (root / MSG_REQUEST).read_text(encoding="utf-8")
    status_msg = (root / MSG_STATUS).read_text(encoding="utf-8")
    check_enum(errors, "Source", cxx_enum(authority, "Source"), msg_constants(request_msg, "SOURCE"), "SOURCE")
    check_enum(errors, "State", cxx_enum(authority, "State"), msg_constants(status_msg, "STATE"), "STATE")
    check_enum(errors, "Reason", cxx_enum(authority, "Reason"), msg_constants(status_msg, "REASON"), "REASON")
    return errors


def main(argv):
    root = pathlib.Path(argv[1]) if len(argv) > 1 else pathlib.Path(__file__).resolve().parent.parent
    errors = check(root)
    for line in errors:
        print("FAIL " + line)
    if errors:
        print("contract check: %d problem(s)" % len(errors))
        return 1
    print("PASS contract check: timing, controllers, ekf, urdf, session.hpp, enums consistent")
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv))
