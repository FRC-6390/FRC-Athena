# Motion Limits and Runtime Utility Contract

This document defines behavior for motion-limit aggregation and small runtime helper utilities.

## MotionLimits

- Base limits and provider limits combine conservatively.
- Positive finite values are considered valid limits.
- Invalid/non-positive values are ignored in combination.
- Effective limit is the smallest valid finite contributor.

## TimedRunner

- First run is always due.
- Non-finite `nowSeconds` is treated as due.
- Zero/negative period runs every check.
- `reset()` clears scheduling history.

## Utility Helpers

- `SupplierUtil.wrap*` returns fallback when source supplier is null.
- `SupplierUtil.optionalDouble/optionalInt` convert invalid values to empty optionals.
- `InputRef<T>` provides mutable late-bound values with `set`, `clear`, and `isSet`.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/core/examples/MotionLimitsExamples.java`
  - `athena-examples/src/main/java/ca/frc6390/athena/util/examples/InputUtilityExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/core/MotionLimitsExamplesTest.java`
  - `athena-test/src/test/java/ca/frc6390/athena/util/InputUtilityExamplesTest.java`
