# athena-localization

Dependency-free localization declarations for Athena 2027.

This module owns pose-estimation configuration, vision measurement weighting,
slip detection thresholds, field bounds, and named autonomous pose aliases. It
does not import WPILib, PhotonVision, Limelight, CTRE, or REV classes.

## Current Slice

- Localization declarations lower through `toSpec()`.
- Vision and multi-tag standard deviations are configurable.
- Generic `VisionFrame` observations can produce weighted pose estimates from
  known AprilTag field poses.
- Slip detection can be tuned or disabled.
- Named field bounding boxes are retained in the spec.
- Autonomous pose aliases point at reusable pose snapshots.
- Validation catches non-finite pose, bounds, weighting, and slip values.

## Dependencies

- Production: `athena-runtime`, `athena-vision`.
- Test-only: none.
