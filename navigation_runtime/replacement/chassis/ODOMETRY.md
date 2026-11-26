# Native wheel feedback / odometry boundary

This is a tested numerical core, not an active ROS2 odometry publisher or TF owner.

## Protocol evidence

- MCU `0800627c` and `080062c8` subtract the saved motor counter and update that
  saved counter. `08006310`/`080063a8` convert that delta before reply packing.
- MCU `0800dee4` packs bytes 11..12 from the left getter and bytes 13..14 with
  the right getter negated. The host must not apply a second right-side reversal.
- Driver `read_odom` (0x48130) divides each signed increment by its corresponding
  left/right PWM scale, truncates to int16, then accumulates it. `publish_ODOM`
  (0x43760) divides these distance increments by 100000. They are not absolute
  encoder counters. Two equal consecutive values can be legitimate equal travel.
- Firmware 441 uses linear raw /1000 and angular raw /-1000 in published ROS
  twist. The original uses /100 for firmware <=437; the candidate refuses other
  versions rather than silently selecting an unverified protocol.
- Current read-only parameters are recorded alongside the verification evidence.
  Left/right scale=1, wheel track=.455 m; original use_imu_data=true.

`verify_feedback_units.py` executes the original x86 instruction slices for wheel
scale/truncation and twist conversion, compares 2000 cases with the compiled C++
implementation, and hashes both sources. This is not full-driver equivalence or
physical calibration. The 1e-5 meter distance divisor is supported by the inspected
publish_ODOM instruction/decompiler output, not a new nonzero distance measurement.

## Pose calculation

`WheelOdometry` is explicitly wheel-only. It uses the constant-curvature SE(2)
exponential, with the straight-line limit, for each pair of wheel increments.
Analytic straight/spin/arc and arc subdivision tests exercise the geometry.
It does not copy factory IMU fusion: factory wheel-only yaw has an additional
pi*wheel_diameter/2 factor, while its enabled IMU branch takes IMU yaw directly.
The candidate uses wheel distance difference / physical wheel track; the scale
and direction must be checked with measured nonzero travel before navigation.

A new explicit epoch starts a new odometry origin. Stream sequence gaps, reversed
or stale receive times and invalid feedback invalidate the estimate without
extrapolating or resuming implicitly. These sequences are transport-side: the
MCU frame has no wire sequence number, so they cannot prove no UART frame was lost.
The native runner must integrate every accepted increment before coalescing ROS
publications; a latest-only raw-increment mailbox would lose distance. It must
also enforce input backlog limits and distinguish receipt time from hardware time.

No map pose, localization correction, IMU sample or TF is mixed into this core.
A later ROS2 estimator should exclusively own odom->base; localization owns
map->odom. Both must have explicit restart epochs. Physical tests remain pending.
