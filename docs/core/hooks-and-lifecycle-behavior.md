# Hooks and Lifecycle Behavior Contract

This document defines expected behavior for `RobotCoreHooks` registration and lifecycle binding semantics.

## Core Rules

- Hook bindings are stored per lifecycle phase in registration order.
- Alias methods map to the same underlying phases:
  - `onTeleInit` == `onTeleopInit`
  - `onAutoInit` == `onAutonomousInit`
  - `onTelePeriodic` == `onTeleopPeriodic`
  - `onAutoPeriodic` == `onAutonomousPeriodic`
- Periodic-loop hooks are tracked with explicit `periodMs`.
- Control loop names must be unique.

## Profile Rules

- Core PID profile output type must be `PERCENT` or `VOLTAGE`.
- Core feedforward profile output type must be `VOLTAGE`.

## Input Rules

- Typed inputs (`bool`, `double`, `int`, `string`, objects) are keyed and immutable once built.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/core/examples/RobotCoreHooksExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/core/RobotCoreHooksExamplesTest.java`
