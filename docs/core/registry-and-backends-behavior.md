# Registry and Auto Backend Behavior Contract

This document defines expected behavior for keyed registries and auto backend selection in core runtime wiring.

## Plugin Registry Contracts

- Registry entries are key/value pairs with non-null key and value.
- `require(key)` returns the value for an existing key.
- Missing keys throw with a clear, registry-specific error message.
- Registry wrappers should expose typed `add(...)` and `require...(...)` methods.

## Auto Backend Contracts

- Backends declare support per `RobotAuto.AutoSource`.
- If multiple backends support a source, highest `priority(source)` wins.
- Default `AutoBackend` methods are safe no-ops and should not throw.
- In core-only classpaths with no vendor modules present, backend discovery may return no backend.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/core/examples/RegistryAndBackendExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/core/RegistryAndBackendExamplesTest.java`
  - `athena-test/src/test/java/ca/frc6390/athena/core/auto/AutoBackendsContractTest.java`
