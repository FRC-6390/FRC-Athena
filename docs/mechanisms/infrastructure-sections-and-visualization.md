# Mechanism Infrastructure, Sections, and Visualization

This document covers the non-behavioral mechanism infrastructure APIs that are easy to overlook:

- section APIs (`sysId`, `networkTables`, `simulation`, `visualization`)
- lazy registerable mechanism factories
- autotune relay-config sanitization
- output conversion helpers
- default visualization coverage across mechanism types

## Section API Contract

- `Mechanism.sysId()` values are mutable at runtime and should be set to safe ranges before binding commands.
- `Mechanism.networkTables()` owner path and publish period are explicit runtime config; no hidden defaults should be relied on in examples.
- test-mode safe behavior should explicitly disable hooks/control loops when needed.

## Registration Contract

- `RegisterableMechanismFactory` should be safe for lazy/cached build patterns.
- `flattenForRegistration()` must reflect the built entry and not duplicate on repeated calls.

## Autotune and Conversion Contract

- Relay autotune configs must be sanitized/clamped by output type.
- Percent/voltage conversions should be explicit and reversible for a fixed battery voltage.

## Visualization Contract

- Vector-line visualization builders should produce a deterministic node name (`<root>VectorLine`).
- Built-in visualization defaults should render non-empty 3D poses for core mechanism types.

## Executable Coverage

- Example: `athena-examples/src/main/java/ca/frc6390/athena/mechanisms/examples/infrastructure/MechanismInfrastructureExamples.java`
- Test: `athena-test/src/test/java/ca/frc6390/athena/mechanisms/infrastructure/MechanismInfrastructureExamplesTest.java`
