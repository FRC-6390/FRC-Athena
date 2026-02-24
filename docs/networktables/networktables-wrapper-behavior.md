# NetworkTables Wrapper and Binding Behavior Contract

This document defines expected behavior for Athena NetworkTables wrappers (`AthenaNT` and `RobotNetworkTables`).

## AthenaNT Rules

- Typed `put/get` helpers map values under `/Athena/...` paths.
- Scoped bindings publish metrics and consume tunables/actions through annotation processing.
- Binding updates occur on explicit tick cycles.

## RobotNetworkTables Rules

- Runtime publishing controls and flags are mutable at runtime.
- Mechanism toggle groups (`details`, `motors`, `encoder`, etc.) apply per-node and persist in the toggle view.

## Executable References

- Example API patterns:
  - `athena-examples/src/main/java/ca/frc6390/athena/networktables/examples/AthenaNTExamples.java`
  - `athena-examples/src/main/java/ca/frc6390/athena/core/examples/RobotNetworkTablesExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/networktables/AthenaNTExamplesTest.java`
  - `athena-test/src/test/java/ca/frc6390/athena/core/RobotNetworkTablesExamplesTest.java`
  - `athena-test/src/test/java/ca/frc6390/athena/networktables/AthenaNTTest.java`
