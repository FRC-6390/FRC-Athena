# ServiceLoader Provider Semantics Contract

This document defines expected behavior for ServiceLoader-backed provider discovery in Athena registries.

## Discovery Rules

- Providers are loaded from the current thread context class loader first.
- If the owner class loader differs, it is scanned afterward.
- Provider de-duplication is class-name based across both scans.
- Registries expose clear missing-provider messages when keys are unresolved.

## Error Rules

- Missing registry keys throw with actionable module guidance.
- Missing hardware factories for explicit type keys throw `IllegalArgumentException` with the missing type key.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/core/examples/ServiceLoaderRegistryExamples.java`
  - `athena-examples/src/main/java/ca/frc6390/athena/hardware/examples/HardwareFactoryExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/core/registry/ServiceLoaderProviderSemanticsTest.java`
  - `athena-test/src/test/java/ca/frc6390/athena/hardware/factory/HardwareFactoryExamplesTest.java`
