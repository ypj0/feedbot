#!/usr/bin/env python3
"""check_contract.py 的测试：真实仓库必须通过，逐项破坏后必须被检出。"""
import importlib.util
import pathlib
import shutil
import tempfile
import unittest

HERE = pathlib.Path(__file__).resolve().parent
ROOT = HERE.parent

spec = importlib.util.spec_from_file_location("check_contract", HERE / "check_contract.py")
check_contract = importlib.util.module_from_spec(spec)
spec.loader.exec_module(check_contract)

FILES = [check_contract.CONTRACT, check_contract.CONTROLLERS, check_contract.URDF,
         check_contract.SESSION_HPP, check_contract.EKF, check_contract.AUTHORITY_HPP,
         check_contract.MSG_REQUEST, check_contract.MSG_STATUS]


class ContractCheck(unittest.TestCase):
    def setUp(self):
        self.tmp = pathlib.Path(tempfile.mkdtemp(prefix="contract-test-"))
        for rel in FILES:
            target = self.tmp / rel
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copy(ROOT / rel, target)

    def tearDown(self):
        shutil.rmtree(self.tmp, ignore_errors=True)

    def mutate(self, rel, old, new):
        path = self.tmp / rel
        text = path.read_text(encoding="utf-8")
        self.assertIn(old, text, "测试前提：被替换的文本必须存在于 %s" % rel)
        path.write_text(text.replace(old, new, 1), encoding="utf-8")

    def assert_detected(self, fragment):
        errors = check_contract.check(self.tmp)
        self.assertTrue(any(fragment in line for line in errors),
                        "未检出 %r；实际：%r" % (fragment, errors))

    def test_real_repository_is_consistent(self):
        self.assertEqual(check_contract.check(ROOT), [])

    def test_update_rate_mismatch(self):
        self.mutate(check_contract.CONTROLLERS, "update_rate: 50", "update_rate: 100")
        self.assert_detected("update_rate")

    def test_publish_rate_mismatch(self):
        self.mutate(check_contract.CONTROLLERS, "publish_rate: 50.0", "publish_rate: 40.0")
        self.assert_detected("publish_rate")

    def test_cmd_vel_timeout_mismatch(self):
        self.mutate(check_contract.CONTROLLERS, "cmd_vel_timeout: 0.2", "cmd_vel_timeout: 0.5")
        self.assert_detected("cmd_vel_timeout")

    def test_wheel_radius_mismatch_in_each_place(self):
        self.mutate(check_contract.CONTROLLERS, "wheel_radius: 0.085", "wheel_radius: 0.09")
        self.assert_detected("controllers.yaml: wheel_radius")

    def test_wheel_radius_mismatch_in_session_header(self):
        self.mutate(check_contract.SESSION_HPP, "radius = 0.085", "radius = 0.0875")
        self.assert_detected("session.hpp: radius")

    def test_wheel_separation_mismatch_in_urdf(self):
        self.mutate(check_contract.URDF, 'xyz="0 0.2275 0.085"', 'xyz="0 0.23 0.085"')
        self.assert_detected("chassis.urdf: left_wheel_joint y")

    def test_odom_tf_must_stay_disabled(self):
        self.mutate(check_contract.CONTROLLERS, "enable_odom_tf: false", "enable_odom_tf: true")
        self.assert_detected("enable_odom_tf")

    def test_ekf_frequency_and_inputs(self):
        self.mutate(check_contract.EKF, "frequency: 50.0", "frequency: 100.0")
        self.assert_detected("ekf.yaml: frequency")
        self.mutate(check_contract.EKF, "imu0: /imu", "imu0: /imu/data")
        self.assert_detected("ekf.yaml: imu0")

    def test_contract_internal_constraints(self):
        self.mutate(check_contract.CONTRACT, "lease_timeout_ms: 300", "lease_timeout_ms: 30")
        self.assert_detected("lease_timeout_ms")

    def test_step_budget_must_fit_in_period(self):
        self.mutate(check_contract.CONTRACT, "control_step_p99_budget_ms: 10",
                    "control_step_p99_budget_ms: 25")
        self.assert_detected("control_step_p99_budget_ms")

    def test_duplicate_topic_names(self):
        self.mutate(check_contract.CONTRACT, "imu: /imu", "imu: /wheel/odom")
        self.assert_detected("重复")

    def test_enum_value_mismatch(self):
        self.mutate(check_contract.MSG_STATUS, "REASON_BUSY=1", "REASON_BUSY=2")
        self.assert_detected("REASON_BUSY")

    def test_enum_missing_constant(self):
        self.mutate(check_contract.MSG_REQUEST, "uint8 SOURCE_DOCK=3\n", "")
        self.assert_detected("SOURCE_DOCK")

    def test_enum_extra_constant_in_msg(self):
        self.mutate(check_contract.MSG_STATUS, "uint8 STATE_FAULT=3\n",
                    "uint8 STATE_FAULT=3\nuint8 STATE_PAUSED=4\n")
        self.assert_detected("STATE_PAUSED")

    def test_cxx_enum_reordering_is_caught(self):
        self.mutate(check_contract.AUTHORITY_HPP, "Busy,             // 已有", "AlreadyHeld,      // 已有")
        errors = check_contract.check(self.tmp)
        self.assertTrue(errors, "重排 C++ 枚举必须被检出")


if __name__ == "__main__":
    unittest.main(verbosity=1)
