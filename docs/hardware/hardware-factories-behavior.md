# Hardware Factories and Registry Resolution Contract

This document defines expected behavior for hardware registry lookups and factory-based device construction.

## Registry Rules

- Motor/encoder/IMU registry lookups are key-based and provider-backed.
- Missing keys throw with clear dependency guidance.

## Factory Rules

- Factory creation requires configs with non-null types.
- Unknown type keys throw `IllegalArgumentException` with the missing type key.
- Factory entrypoints wrap raw vendor devices with Athena adapters.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/hardware/examples/HardwareFactoryExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/hardware/factory/HardwareFactoryExamplesTest.java`
