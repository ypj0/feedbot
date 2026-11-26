# Native chassis execution core and stationary commissioning

`protocol.hpp` encodes a **single frozen reviewed hardware profile**, with only
linear/angular target fields and checksum allowed to change. The stationary frame
matches all 150 frames in the historical real TX recording. Current read-only ROS
parameters also match its wheel diameter 0.17 m, wheel track 0.455 m, gear parameter
5600, and tick_meter_ratio 10. Unknown auxiliary bytes are frozen, not repurposed.
Feedback speed and encoder fields remain explicitly raw; no odometry units are
invented. This does not prove every configuration byte's physical meaning.

`lease.hpp` separates clock/command expiry from receiving feedback. Tick produces
a complete stationary frame even if receive processing never delivers another event.
New owner epochs and increasing command sequences reject old requests without
interrupting a valid current owner. Expiry, unhealthy feedback, clock reversal or
invalid current-owner values revoke motion; subsequent valid feedback cannot resume
old commands. Reset requires fresh physical emergency feedback and zero raw speed.

Sanitizer tests cover exact stationary bytes, full-frame invariance, numerical
validation, expiry without RX, stale owner rejection, emergency and explicit reset.

`transport.hpp` adds a nonblocking, single-owner reactor over an already owned fd.
Each cycle has a bounded receive budget (1024 bytes); transmit scheduling does not
wait for a feedback packet. Its 20 ms output cadence is a host scheduling target,
not a measured MCU stopping deadline. A fixed 40-byte parser resynchronizes after
corrupt data without growing memory. Pending output has a 40 ms deadline. An
expired or changed target in a partly written frame is a terminal transport fault:
it must not be spliced into a different command. A transport fault cannot claim
that the MCU received a stop. The caller must service the reactor on a dedicated
thread; that production runner is not implemented yet.

Command expiry is checked when accepting a new command as well as during TX,
so a delayed producer cannot renew an expired owner between scheduler ticks.

Run `python3 navigation_runtime/replacement/chassis/run_tests.py` on the local
machine. ASan/UBSan suites cover Unix stream corruption, absent RX, blocked TX,
short writes, expiry during partial output, restart disarmed, and an actual PTY
with byte-wise fragmented input and disconnect. These tests open only temporary
PTYs and socket pairs; no robot device is accessed. UART ownership is not implied
by possession of an fd. A future serial runner must use raw mode, VMIN=1,
VTIME=0 and O_NONBLOCK, establish exclusive ownership before opening the device,
and ensure its fd lifecycle cannot be reused by another thread. Socket-based
harnesses must handle SIGPIPE (the production target is a tty).

**This is not a deployed hardware driver or an active motion endpoint.** No ROS
publisher or web motion endpoint is supplied. A separate stationary maintenance
probe has now been deployed and tested; it does not expose motion commands. Dedicated
reactor scheduling, exclusive device ownership, startup/shutdown handshake,
kernel/device buffering bounds, backlog freshness, MCU timeout and physical
stopping behavior remain to be implemented/validated.
Do not replay the frozen frame onto a different chassis or assume a host-side lease
can stop motors after the host/USB/MCU fails. This code was not sent to the chassis.


## Stationary hardware commissioning tool

`stationary_probe.cpp` is deliberately a separate maintenance executable: it has
no arming, velocity command, charging, undocking or arbitrary packet API. It uses
the same fixed-memory frame decoder as the runtime transport. At 115200 baud it
sends the recovered type-1 handshake, checks the 0x60 response and returned baud,
then sends only the reviewed stationary frame for ten seconds. Every runtime
response must report emergency=1, initialized=1, firmware=441 and both raw speeds=0.
A 200 ms receive timeout, 40 ms write deadline, nonempty kernel output queue,
interruption or malformed runtime response ends the test. These are maintenance
checks, not proof of physical stopping after a moving command.

The Linux hardware mode audits all other process descriptors by device number,
requires TIOCEXCL and repeats the owner audit after opening. TIOCEXCL does not
exclude privileged reopeners; the worker must also stop the factory service.
The original termios is restored on normal/error scope exit. Input backlog is
flushed before the handshake; output is never spliced/flushed into another frame.

`deploy_stationary.py` without arguments uploads, compiles with the chassis GCC,
and runs five disconnected PTY cases. `--run` requires fresh emergency/stationary
factory telemetry, launches an independent systemd worker, stops factory service,
runs the probe, restores the factory and verifies fresh scan/odom/driver feedback.
A separate 200-second restore timer covers worker death; the worker has a
180-second systemd runtime limit. The manager source must retain its reviewed
SHA256. No factory file is edited. The maintenance observer must be running.
Do not use this tool as a navigation service or release the emergency during it.


### Actual stationary trial, 2026-11-07

On the internal chassis computer, the bounded probe acquired `/dev/rikibase`
(`/dev/ttyUSB1`) after factory shutdown, completed the 115200 handshake, and
exchanged 494 stationary requests and 494 runtime responses in ten seconds.
Maximum feedback interval was 0.0213695 s. Every accepted response reported
physical emergency=1, initialized=1, version=441 and both raw speeds=0. There
were no nonzero velocity requests or action commands.

Factory service restarted, but the first verification timed out because the
read-only observer retained subscriptions to the old ROS master. This is a
monitor lifecycle failure, not evidence of missing hardware feedback. Recovery
now explicitly recreates that observer after starting the factory. Applying
that recovery step restored fresh driver/odom/1075-beam scan evidence. The
original manager SHA256 remained unchanged. The independent restore timer was
cancelled only after that evidence was verified. Initial failure logs are kept.

Evidence: `analysis/body_navigation_20261107/readiness/chassis_stationary/`.
This verifies stationary ownership/handshake/return-to-factory. It does not
validate nonzero motion, physical stop distances, encoder units, or fail-safe
stopping when the host dies. No full driving trial is ready yet.
