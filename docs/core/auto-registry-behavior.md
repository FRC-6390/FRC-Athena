# Auto Registry Behavior Contract

This document defines expected behavior for runtime auto-engine provider lookup.

## Core Rules

- Providers register engines under a string key.
- Looking up a registered key returns the associated `RobotAuto.AutoSource`.
- Looking up a missing key throws with a clear dependency guidance message.
- Registry access uses a shared singleton (`AutoRegistry.get()`).

## Error Contract

When key lookup fails, error text must indicate:

- Which key was missing.
- That an appropriate `athena-*` auto module dependency is required.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/core/examples/AutoRegistryExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/core/AutoRegistryExamplesTest.java`
