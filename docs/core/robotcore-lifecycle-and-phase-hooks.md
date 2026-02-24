# RobotCore Lifecycle and Phase Hooks Contract

This document defines expected behavior for lifecycle-phase hook registration and dispatch.

## Phase Dispatch Rules

- Hooks are grouped by explicit lifecycle phases (`ROBOT_INIT`, `TELEOP_PERIODIC`, etc.).
- Running a phase executes only bindings registered for that phase.
- Non-periodic phases must not run periodic-loop bindings.
- Binding order is deterministic and matches registration order.

## Periodic Loop Rules

- Periodic-loop hooks are scoped to periodic phases only:
  - `ROBOT_PERIODIC`
  - `DISABLED_PERIODIC`
  - `TELEOP_PERIODIC`
  - `AUTONOMOUS_PERIODIC`
  - `TEST_PERIODIC`
- Each loop binding carries a finite non-negative `periodMs`.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/core/examples/RobotCoreLifecycleExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/core/RobotCoreLifecycleExamplesTest.java`
