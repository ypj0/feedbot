# Native WIT IMU decoder — offline / isolated tests only

`wit_decoder.hpp` decodes the original driver's 11-byte 0x55/0x51..53 format,
checks the sum, reads signed little-endian channels and converts to m/s², rad/s
and roll/pitch/yaw radians with a ROS-order x/y/z/w quaternion. Formula evidence:
`analysis/decompiled_full/imu_node/00107580.c` (SingleIMU::readIMU).

Fixed memory replaces the original unbounded receive vector. Corrupted frames
resynchronize one byte at a time. Acceleration, gyro and orientation have separate
receipt timestamps and readiness requires all three to meet explicitly supplied
age and skew bounds. No channel update refreshes another channel's age. Receipt
time is not a hardware timestamp; a production reader must bound serial backlog.
The header does not open/configure the sensor, calibrate axes, remove gravity,
fuse measurements or publish TF. Other WIT packet types are not decoded.

Local ASan/UBSan tests cover gravity/gyro units, 90-degree yaw quaternion, signed
limits, fragmented input, corrupted frames, stale channels and reversed clocks.
Internal chassis GCC build/tests are also recorded in the chassis_odometry
readiness evidence. No physical IMU serial handoff or nonzero calibration has
been performed for this decoder.
