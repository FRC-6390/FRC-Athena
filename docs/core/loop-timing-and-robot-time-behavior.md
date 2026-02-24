# Loop Timing and Robot Time Behavior Contract

This document defines expected behavior for per-cycle timing cache and mechanism timing sampling.

## Core Rules

- `LoopTiming.beginCycle()` caches one loop timestamp via `RobotTime.nowSeconds()`.
- `LoopTiming.setDebugAlways(true)` enables mechanism sampling every cycle.
- `LoopTiming.recordMechanism(...)` records durations only while sampling is active.
- Repeated mechanism records in one cycle accumulate into the same mechanism bucket.

## Safety and Diagnostics

- When sampling is disabled, mechanism records are ignored.
- Timing APIs should be safe to call in any loop phase and should not require external synchronization.

## Executable References

- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/core/LoopTimingAndRobotTimeTest.java`
