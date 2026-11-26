# IMU publishing latency: next evidence boundary

## Confirmed chain

Run02 and run03 retain complete Docker stderr in their `remote_results/commands.log` files. The IMU publishing expression took respectively 333.156370 ms and 333.443712 ms after a valid gyro was decoded. The unchanged 200 ms freshness check then failed, the IMU exited, the stack shut down, and the lidar activation probe returned 137. Lidar activation failure is downstream, not evidence of a new lidar protocol fault.

The timed expression includes ROS clock access, message construction and publication. Wall duration alone does not distinguish execution, blocking or scheduler delay. Two similar durations are a pattern, not proof of a fixed middleware timeout. Neither run proves that hardware kept transmitting throughout the interval, nor establishes a shared cause with the historical MCU timeout.

Run03 restored factory operation, but its observer exited about 18.7 seconds before the fault. Its zero lost-event counters do not establish fault coverage. Original run artifacts remain unchanged.

## Resolve the observation failure first

The old recorder's lifetime depended on periodic Docker, `/proc` and serial-owner queries. A factory process exiting during the expected handoff caused `/proc/17771/exe` lookup to fail and stopped the entire recorder. Adding a local catch would retain this unnecessary dependency and the measured 175-267 ms polling cost per second.

The replacement records only kernel events in an independent trace instance:

- `sched_switch`: either `prev_comm` or `next_comm` is `imu_node`.
- `sched_wakeup` and `sched_wakeup_new`: wakee `comm` is `imu_node`.
- `sched_process_exit`: `comm` is `imu_node`.
- `sched_process_exec`: filename is `/work/install/lib/feedbot_imu_ros2/imu_node`.

There is no periodic process enumeration, Docker inspect, serial query, syscall tracing or metadata sampler. Existing handoff and restoration checks retain responsibility for device ownership. Recorder failures remain explicit; no retries or alternate collection modes are introduced.

## Evidence interpretation

Select the native main host PID from its exact executable event, then correlate only that PID's events with the application's monotonic timestamps. A matching process name alone is insufficient: factory IMU uses the same name. Require the executable event before the fault and coverage through the end of the timed expression, with no relevant loss or overrun. Missing evidence means unclassified, not normal scheduling.

- Switch-out while runnable, followed by delayed switch-in: runnable off-CPU interval.
- Switch-out in a sleep state: blocked/sleeping interval; wakeup to switch-in separates subsequent runnable delay.
- Switch-in to switch-out: scheduled-on-CPU interval; this alone does not prove useful execution or DDS work, and does not exclude interrupt time.

This first collection distinguishes scheduler state only. It cannot name a lock, system call, DDS operation or hardware fault. A later narrower observation is justified only by the measured state.

## Local implementation and verification

`agv_workspace/acceptance/record_imu_schedule.py` implements the scheduler-only recorder with a fixed 90-second upper bound and explicit SIGTERM stop. `recorder_ready.json` identifies completed setup; the final manifest, trace and loss statistics are still required to assess capture coverage. It uses the previously verified host tracefs path `/sys/kernel/debug/tracing`, not an assumed alternate mount.

Local `py_compile` and 11 tests in `test_record_imu_schedule.py` pass. Tests cover setup/filter failure, interruption before and during readiness, normal bounded completion, unexpected drain exit, missing statistics, preservation of existing artifacts/foreign instances and absence of `/proc` dependencies. Python 3.6 syntax compatibility was checked locally; actual host execution and kernel filter acceptance remain untested. No driver or protocol file changed, and no vehicle was reactivated during this preparation.

## Before another vehicle window

1. Pass local recorder tests, including output preservation, setup failure, early drain failure and bounded cleanup. This is code evidence only.
2. With factory services untouched, verify actual host event formats, filter acceptance/readback and monotonic clock selection in an independent instance. Filter rejection stops preparation; it must not trigger an unfiltered substitute.
3. Obtain authorization for one zero-command session. Start the recorder before native executable startup. Do not change protocols, thresholds, clocks, priorities, DDS settings or stack launch order.
4. Correlate full stderr and native PID scheduler events, then verify factory restoration. Recording completion without fault reproduction establishes a baseline only, not a fix.

Current vehicle integration status: **未通过**. The previous window's one activation is consumed; this preparation does not authorize another activation.
