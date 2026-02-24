# Vendor Module Service Contracts

This document defines the contract for Athena vendor-module discovery via `ServiceLoader`.

## Why This Exists

Athena relies on `META-INF/services` descriptors in vendor modules (`athena-ctre`, `athena-rev`, `athena-studica`, camera modules, and auto-engine modules).
If a descriptor entry drifts, runtime registry/factory lookups fail with missing-provider errors.

## Contract

- Each vendor module must publish service descriptors under `src/main/resources/META-INF/services`.
- Each descriptor must list the expected implementation class names.
- Each listed implementation must exist in `src/main/java`.
- String-backed registry providers must publish stable keys:
- `auto:pathplanner`
- `auto:choreo`
- `camera:photonvision`
- `camera:limelight`

## Executable Coverage

- Example: `athena-examples/src/main/java/ca/frc6390/athena/core/examples/VendorModuleExamples.java`
- Test: `athena-test/src/test/java/ca/frc6390/athena/core/registry/VendorModuleServiceContractsTest.java`
