# Hardware Acceptance Coverage

`hardware_test_matrix.json` is the authoritative machine-readable gap matrix.
Every complete device-acceptance result remains `unknown`; selected offline and
read-only checks have now run. See `planning/hardware_validation_20261122.md`
and linked run evidence. The live manifest fingerprints collected files but records
incomplete source/command provenance; it is not immutable storage or a completed
run-record contract. Partial observations never imply hardware PASS.

## Safe Offline Baseline

| Area | Existing command or source | Covered | Not covered |
| --- | --- | --- | --- |
| Archive | `python3 校验资料包.py` | Archived file hashes/sizes | Hardware state or behavior |
| Recovered model | CMake under `navigation_runtime/reverse/recovered` | Recovered C semantics build | Firmware equivalence or hardware I/O |
| Chassis | `python3 navigation_runtime/replacement/chassis/run_tests.py` | Frame/profile, lease, bounded RX, no RX, corrupt RX, blocked/partial TX, PTY disconnect, odometry/stationarity tests | Serial ownership in production, MCU watchdog, physical stop, brakes, motors, charge hardware |
| IMU | `navigation_runtime/replacement/imu/test.cpp` | WIT decoding, units, resync, timestamps/readiness | Device lifecycle, model identity, mounting, calibration, physical timing |
| Lidar parser | `navigation_runtime/replacement/lidar/{test.cpp,replay.cpp,lifecycle_test.cpp}` through its ROS2 CMake project | Recorded raw frame parsing and lifecycle core | Real SDK/device control, installation calibration, owner handoff |
| Lidar fixtures | `test_supervisor.py`, `sdk_fixture.py`, `managed_fixture.py` | Transaction/lifecycle and loopback SDK behavior | Factory-process control or real lidar behavior |

## Non-default Entrypoints

- `chassis/deploy_stationary.py` is a remote staging/maintenance action, not an
  offline test. It depends on missing `replacement/tools/onboard_admin.py`; its
  `--run` branch stops the factory service and starts systemd workers.
- `lidar/run_acceptance.sh` and `lidar/run_control_fixture.sh` run on the
  onboard container path `/opt/feedbot-nav2/lidar`; they build and write there.
- Lidar collection/deployment scripts also depend on the absent
  `replacement/tools/onboard_admin.py`; do not use them as local defaults.
- `chassis/verify_feedback_units.py` requires Unicorn and pyelftools and writes
  an archival report. Run only from a working copy after preserving provenance.

## Required Evidence Before PASS

- G0: Actual equipment only—nameplates, serials, power/wiring, mount and purpose.
- G2: Current passive identity, owner, freshness and raw evidence; preserve owners.
- G3: Fresh physical E-stop assertion and zero feedback are mandatory before any
  serial or lidar ownership handoff. Current `EmergencyStatus=0` blocks G3.
- G4: Approved off-dock controlled motion, external stop measurements, failure
  injection, latch/reset evidence, and no automatic recovery of old motion.
- G5/G6: All installed hardware PASS plus a buildable ROS2 package, launch/config,
  testdata and production regression coverage. New acceptance tools and an incident
  ledger do not fill the missing production ROS2 package or safety integration.

## Current Docked Condition

The vehicle is reported stopped on its charging dock with on-site watchers, but
the user declines pressing physical E-stop and feedback remains zero. Do not send nonzero velocity,
undock, or alter charging state. New evidence belongs in
`agv_workspace/acceptance/runs/<UTC-run-id>/`; do not rewrite archived evidence.
