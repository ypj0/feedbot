# IMU timing observations

The adapter reports one-second diagnostic aggregates without logging each frame. All fields ending in
`steady_s` and all durations use the process monotonic clock; the ROS message stamp remains the existing
`now()` value taken when a valid gyro is decoded, before publication handoff. The worker preserves that
stamp; it does not restamp the message when DDS publication completes. Absolute time alignment is unverified.

The run75 candidate calls `receive()` from one dedicated receiver thread, followed by a 2 ms sleep,
instead of a ROS executor timer. Existing `callback_*` field names now refer to consecutive receiver-loop
entries. Their spacing includes receive processing, the sleep and OS scheduling, not an exact 2 ms period.
This removes executor wait dependence, not OS scheduling delays; IMU, EKF and lidar remain separate processes.
The original 157 ms event has not been attributed to an executor wait.

The run77 latest-value policy replaces the IMU realtime publisher's reject-new handoff with one pending
message and a separate publication worker. A new valid sample replaces an unconsumed pending sample;
an already in-flight DDS call cannot be replaced. The worker takes the pending value under a short mutex
and calls DDS outside that mutex. The receiver never waits for DDS completion, but this is not a hard
real-time or lock-free scheduling guarantee. No FIFO, retry, old-frame repetition or threshold change is added.

`publish_policy=latest_pending` identifies this contract. `publish_handoff_count` counts accepted mailbox
submissions, NOT DDS completion. `publish_pending_overwrite_count` counts displaced pending samples;
`publish_completed_count` counts successful returns from DDS publish, NOT subscriber delivery. `publish_pending`
and `publish_in_flight` identify the two possible outstanding samples. `publish_discarded_on_stop_count` counts
pending samples discarded at shutdown/failure; `publish_failure_count` counts a thrown publication call.
Their sum equals submissions in each diagnostic snapshot. Overwrites remain WARN and are sampling loss,
not lossless delivery. The obsolete `publish_handoff_drop_count` is removed rather than publishing a constant
zero that older acceptance tools could mistake for lossless publication. Historical reports retain old semantics.
Diagnostics still use their existing realtime publisher and separate `diagnostic_handoff_drop_count`.

Shutdown discards the pending value and joins an in-flight publication; it cannot cancel a blocked DDS call.
A publication exception is retained and checked on every receiver iteration, propagated through the existing
error/serial-release/exit-1 path without retry. The worker is joined before its publisher is destroyed.
`tests/latest_imu_publisher_test.cpp` exercises a gated publication with 9 newer arrivals: only the first
in-flight and final newest sample publish, with 8 explicit overwrites and unchanged stamps and covariance.
`tests/latest_handoff_fixture.py` checks paced, burst and delayed-publication PTY input, counter accounting,
newest-sample delivery, no repeats and normal serial release.

Each one-second diagnostic also reports `callback_gap_histogram_sample_count` and
`callback_gap_histogram_counts`. The counts contain 107 comma-separated buckets in this fixed order:
100 buckets `[0.0, 0.1)`, `[0.1, 0.2)`, ..., `[9.9, 10.0)` ms, followed by `[10, 20)`, `[20, 50)`,
`[50, 100)`, `[100, 200)`, `[200, 500)`, `[500, 1000)`, and `[1000, +inf)` ms. An interval exactly
on a boundary belongs to the bucket on its right. The first callback has no preceding callback and is not
sampled. Every later callback interval contributes to exactly one bucket, and the window resets after each
diagnostic report. The sample count therefore equals the sum of all counts.

Merged windows can locate P50/P95/P99 only within one of these intervals; the histogram does not retain raw
samples and must not be reported as exact percentiles. Callback gaps measure host wall-clock execution spacing,
including scheduler delay, callback processing, and any blocking in the execution path. They do not by themselves
attribute a long interval to thread starvation or to any single component.

`parse_excluding_publish` is wall time from serial read return through byte decoding and adapter bookkeeping,
minus the measured calls that construct and publish valid gyro messages. It is not decoder-only CPU time.
`gyro_read_return_batch_gap` compares the read-return times of consecutive non-empty reads containing at least
one valid gyro packet. It does not include zero-valued intervals for additional packets from the same read and
is not a hardware byte-arrival interval.

The serial descriptor remains `O_NONBLOCK` with `VMIN=1` and `VTIME=0`; this is not a 50 ms timed read.
`read_mean_s` and `read_max_s` measure the wall duration of individual `read(2)` calls. A callback that gets
`EAGAIN` or an incomplete gyro packet still performs the unchanged 200 ms timeout check, while a hard read or
disconnect error fails earlier. A valid gyro received after a delay refreshes the timeout origin before that
callback's check; callback and gyro-batch gap fields still retain the delay, but their trend alone cannot assign
the delay to the device rather than scheduling or the read path.

On failure, the node closes the serial port, publishes the error diagnostic, and emits the most recent 128
callback batches. The receiver retains the exception and shuts down the ROS context to wake the executor.
Main joins the receiver before rethrowing that exception, logging its reason and returning exit code 1.
Normal shutdown also joins the receiver before destroying the port or publishers. Each journal
entry includes callback entry/gap, read boundaries, queued/read byte counts, valid gyro and cumulative decoder
accepted/rejected counts, parse/publish durations, and the exact `last_gyro` origin and check time used for the
unchanged 200 ms timeout. At the nominal 2 ms callback period the ring spans about 256 ms, but scheduler delays
mean that coverage duration is not guaranteed. The ring contains only receive batches completed through the
timeout-check point, including the batch that triggers a 200 ms timeout. An earlier ioctl, backlog, or read failure
has no current-batch entry; the ring is not padded with a synthetic completed batch.

A separate `IMU failed attempt` record preserves that failing attempt: stage, callback entry/gap, queue-query
boundaries/result/errno and actual queued bytes, read-attempt flag/boundaries/result/errno, and failure-observed
time. These values are captured before cleanup and error publication, although the log itself follows the
existing error publication. Error diagnostics expose the same attempt as `failure_*` fields. Queue value `-1`
means the query did not succeed. When `read_attempted=0`, zero read timestamps are absent observations, not a
zero-duration successful read. `timeout_check` remains the last executed check; at `queue_limit` it belongs to
the preceding completed callback, not the failing attempt. All times are host monotonic observations, not
hardware byte-arrival times. Two clock reads surround the queue query; no new per-byte timing is introduced.

The isolated `tests/backlog_fixture.py` deliberately pauses only its container PTY node, injects 2112 bytes,
and resumes it once. It checks the unchanged backlog failure, one separate failed-attempt record, the ordered
128-entry completed ring and descriptor release. `tests/observation_fixture.py` covers normal reports, bad
checksums and the unchanged 200 ms stop-stream exit. These tests validate observation boundaries, not the
cause of a vehicle backlog. Neither the 1024-byte limit nor the 200 ms timeout, parser, or exit behavior changes.

`tests/executor_isolation_fixture.py` with `tests/executor_delay.c` injects a single 400 ms delay into
`rcl_wait` inside an isolated PTY container. The timer baseline fails its existing backlog limit; the receiver
candidate must continue publishing during the delay and exit normally on SIGTERM. This proves an artificial
executor-dependency contrast, not that the historical delay had the same cause. Stop-stream and process-wide
SIGSTOP/backlog tests must still fail explicitly with exit code 1; isolating the executor does not bypass them.

Future vehicle EKF load acceptance should record CPU and thread counts for 10 seconds before and 30 seconds after
EKF activation. That load window is not part of this isolated PTY verification.
