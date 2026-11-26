# MCU ros2_control adapter

## Fault release and TX evidence

Fault and lifecycle cleanup share `release_connection.hpp`: attempt the existing
stop once, close the serial owner, then report errors using the retained Session
snapshot. Stop and diagnostic errors remain explicit. Fatal diagnostic publication
is still synchronous after close; this is not a fully nonblocking callback.

Session retains 128 write attempts in fixed memory. Fault diagnostics export
`tx_history_json` with intended frame bytes, syscall offset/length/result/errno,
monotonic entry/exit times and command/stop provenance. Normal diagnostics expose
`tx_attempt_count`; no per-write DDS or file output is added. Sequence numbers
identify overwritten history. Handshake writes are outside this record, and a
successful host write does not prove MCU receipt or physical stopping.

Jazzy compile and five CTests pass in
`agv_workspace/acceptance/runs/20261123_chassis_release_tx_31/RESULT.md`.
This candidate is not deployed or hardware-validated. The firmware ACK=0x80
request-validation failure branch is identified; its live triggering cause remains
open. Protocol encoding, deadlines and fail-on-error behavior remain unchanged.

Jazzy package `feedbot_chassis_ros2`, plugin `feedbot_chassis_ros2/ChassisSystem`.
Runs on the x86 car computer, not RK3588. This is a navigation integration adapter,
not a new MCU protocol implementation.

## Frozen inputs and units

Reuse the archived `navigation_runtime/replacement/chassis/` headers unchanged:
`protocol.hpp`, `serial_port.hpp`, `frame_stream.hpp`, and `odometry.hpp`.
`ODOMETRY.md` documents the original `read_odom`/`publish_ODOM` conversion and
the firmware 441 per-response increments; `odometry_test.cpp` tests that conversion.

With the observed left/right scale of 1, `wheel_sample()` converts each signed
increment to metres by dividing by 100000. The adapter divides that distance by
the nominal wheel radius, 0.085 m, and accumulates **every** decoded packet.
The equivalent count per revolution is `2π × 0.085 × 100000 ≈ 53407.0751`.
This is a distance-unit equivalent, **not an encoder line count**. The frozen
configuration byte `tick_meter_ratio=10` is not evidence for a separate encoder
resolution. Scale and direction still require passive-wheel and ground validation.
The right-wheel sign is already corrected by the MCU and is not negated here.

`write()` converts controller wheel commands using
`v=0.085(left+right)/2`, `ω=0.085(right−left)/0.455`, then calls
`velocity_frame()`. It never consumes `/cmd_vel` directly. Nominal dimensions
in the adapter, URDF and controller YAML must be updated together after calibration.

## Lifecycle and failure behavior

- Configure creates diagnostics; activate acquires `/dev/rikibase`, runs the
  validated 115200 8N1/type-1/0x60 handshake, sends zero and waits for valid feedback.
- Active read drains all available bytes through the frozen parser. Incomplete
  frames are retained by that parser, never filled or published as complete data.
- Deactivate sends zero and releases the serial port; cleanup removes diagnostics.
  Explicit reactivation creates a new position origin and clears commands.
- A 200 ms feedback timeout, invalid feedback or write error returns `ERROR`,
  clears commands, closes the port and leaves the component faulted. Reload is
  required; there is no reconnect or old-command replay. Writes have a 40 ms
  deadline. Following output failure no stop frame is appended to partial output.
- Emergency/charging/alarms block nonzero commands. Diagnostics on `/diagnostics`
  expose the original fields, feedback count and rejected-byte count separately
  from wheel position. A successful host write is not proof of physical stopping.

## Build and disconnected verification

Inside the frozen `feedbot-navigation:jazzy-20261122` image, mount the bundle at
`/work` (or provide `-DCHASSIS_CORE=/path/to/frozen/headers`):

```sh
source /opt/ros/jazzy/setup.bash
cmake -S /work/agv_workspace/drivers/chassis_ros2 -B /work/build -DCMAKE_INSTALL_PREFIX=/work/install
cmake --build /work/build -j2
ctest --test-dir /work/build --output-on-failure
cmake --install /work/build
source /work/install/share/feedbot_chassis_ros2/local_setup.bash
python3 /work/agv_workspace/drivers/chassis_ros2/tests/controller_fixture.py /work/fixture_results
```

The controller fixture must run in a container **without any robot device mapping**.
It creates a temporary PTY at `/dev/rikibase` and exercises the real plugin,
controller manager, diff-drive controller, diagnostics and odometry. Its covariance
values are synthetic test inputs, not an approved hardware configuration.

## Physical commissioning prerequisites

`ros2 launch feedbot_chassis_ros2 chassis.launch.py covariance_file:=/path/to/measured.yaml`
requires an explicit YAML file with `wheel.ros__parameters` entries
`twist_covariance_diagonal` and `pose_covariance_diagonal`, each a six-element
variance list ordered x/y/z/roll/pitch/yaw. Keep measurement provenance beside it.
There are intentionally no invented physical defaults. Verify odometry indices
0 and 35 at runtime; initial EKF does not fuse wheel pose. **Physical covariance
initialization remains pending measured/modelled evidence.**

Before a real run, arrange a separate factory-service handoff and recovery window.
Use the host PID namespace (`--pid host`) so the frozen `/proc` owner audit sees
factory processes, and map only the required serial device. Never run beside the
factory serial owner. This package does not stop or restart `spirit.service`.

For passive-wheel acceptance, obtain current confirmation of reliable support
and no motor torque. Record left-only, right-only and same-direction rotation,
joint position and `/wheel/odom.twist`, runtime `open_loop=false`,
`position_feedback=true`, `enable_odom_tf=false`, diagnostic/physical status
agreement, exclusive ownership and release. Static software zeros do not establish
motor torque-off. No passive-wheel or ground-travel acceptance is claimed here.
