# Controllers and Filters Behavior Contract

This document defines expected behavior for core controller helpers and filter wrappers.

## Controller Helpers

- `ModifiedAxis` applies deadzone shaping before optional squaring and inversion.
- `Debouncer` only emits true after its period condition is satisfied.
- `DelayedOutput` requires input to remain true for the configured delay window.

## Filter Helpers

- `FilteredValue` supports chainable filter pipelines and cached last-value reads.
- `FilteredPose` applies axis-wise filter chains and returns filtered pose snapshots.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/controllers/examples/ControllerHelperExamples.java`
  - `athena-examples/src/main/java/ca/frc6390/athena/filters/examples/FilterExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/controllers/ControllerHelperExamplesTest.java`
  - `athena-test/src/test/java/ca/frc6390/athena/filters/FilterExamplesTest.java`
