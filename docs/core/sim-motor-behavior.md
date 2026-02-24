# Simulation Motor Behavior Contract

This document defines expected behavior for vendor-agnostic simulation motor factories.

## Motor Model Contract

- `Motor` enum values provide simulation-ready `DCMotor` models.
- `MotorSimType` custom factories can be used for caller-defined simulation models.
- Helper wrappers should normalize motor count to at least one motor.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/sim/examples/MotorSimExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/sim/MotorSimExamplesTest.java`
