# RobotSpeeds Behavior Contract

This document defines expected behavior for source blending and output arbitration in `RobotSpeeds`.

## Core Rules

- Every source contributes independently on `X`, `Y`, and `Theta`.
- Source-level enable/disable gates contribution on all axes.
- Axis-level enable/disable gates contribution per axis.
- Output is clamped by `maxVelocity` and `maxAngularVelocity`.

## Blend Pipeline

1. Raw source inputs are collected.
2. Source-to-source blends are applied in registration order.
3. Output blends are applied in registration order.
4. Final values are clamped.

## Safety and Validation

- Unknown source names throw an error.
- Circular source blend dependencies are rejected.
- Division blend with near-zero divisor preserves the left value.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/core/examples/RobotSpeedsExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/core/RobotSpeedsExamplesTest.java`
