# Auto Selection and Execution Behavior Contract

This document defines expected behavior for chooser-driven `RobotAuto` selection and command retrieval.

## Chooser Contracts

- Program chooser requires at least one registered routine.
- Default chooser id must reference an existing routine.
- Command chooser maps selected command back to the selected routine metadata.

## Execution Contracts

- `execution().selectedCommand()` returns the selected command when a chooser has been configured.
- `execution().prepare()` finalizes registrations and should be safe to call during startup.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/core/examples/AutoExecutionExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/core/AutoExecutionExamplesTest.java`
