#!/bin/sh
# Offline only. Usage: sh .../verify_all.sh /path/to/venv/bin/python
set -eu
cd "$(dirname "$0")/../.."
PYTHON=${1:-python3}
BUILD=$(mktemp -d "${TMPDIR:-/tmp}/feedbot-recovery.XXXXXX")
trap 'rm -rf "$BUILD"' EXIT HUP INT TERM
cc -std=c11 -Wall -Wextra -Werror -ffp-contract=off -shared -fPIC navigation_runtime/reverse/recovered/*.c -o "$BUILD/models.so" -lm
for TEST in verify_charge_planar verify_charge_execute verify_driver_smoothing verify_driver_frame verify_dwa_pid verify_lidar_packet verify_recovery verify_sub_target verify_sub_kinematics verify_motor_output verify_sub_motion_chain; do
  "$PYTHON" "navigation_runtime/reverse/$TEST.py" "$BUILD/models.so"
done
"$PYTHON" navigation_runtime/reverse/audit_mcu_startup.py
