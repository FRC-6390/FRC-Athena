# IMU and Encoder Edge-Case Contract

This document defines expected edge-case behavior for IMU and encoder wrappers.

## IMU Rules

- Virtual IMU inversion applies consistently to simulated orientation and angular velocity.
- Non-finite max-speed window updates are ignored.
- Max-speed window duration is clamped to a safe minimum.
- Angular acceleration reporting remains finite during normal update progression.

## Encoder Rules

- Encoder adapter position writes remain finite even when conversion or gear ratio inputs are zero.
- Absolute position discontinuity settings wrap absolute rotations into the configured range.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/hardware/examples/ImuEncoderEdgeExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/hardware/imu/ImuEdgeBehaviorTest.java`
  - `athena-test/src/test/java/ca/frc6390/athena/hardware/encoder/EncoderEdgeBehaviorTest.java`
