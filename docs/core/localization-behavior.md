# Localization Config Behavior Contract

This document defines expected behavior for `RobotLocalizationConfig` builder usage.

## Config Contract

- Vision standard deviations and multi-tag deviations are configurable through estimation/config sections.
- Slip/backend tuning values are configurable via backend sections.
- Per-pose config supports section-style builders (`poses().pose("name", pose -> ...)`) including per-pose backend overrides.
- Slip detection can be explicitly disabled via backend config (`slipDetectionEnabled(false)` / `slip(s -> s.enabled(false))`).
- Legacy pose factory/fluent APIs (`poseConfig(...)`, `pose(PoseConfig)`, `PoseConfig.*()` presets, `PoseConfig.with*`) are removed.
- Named bounding boxes are retained and auto-pose aliases can be assigned.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/core/examples/LocalizationExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/core/localization/LocalizationExamplesTest.java`
  - `athena-test/src/test/java/ca/frc6390/athena/core/localization/RobotLocalizationConfigBoundingBoxTest.java`
  - `athena-test/src/test/java/ca/frc6390/athena/core/localization/RobotLocalizationSlipSolverTest.java`
