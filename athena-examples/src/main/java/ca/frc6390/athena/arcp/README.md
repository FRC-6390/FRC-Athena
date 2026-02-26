# ARCP Examples

This folder contains ARCP-specific usage examples, separate from `core/examples`.

## Location

- `athena-examples/src/main/java/ca/frc6390/athena/arcp/examples`

## What Is Covered

- Signal publishing (`put` overloads for primitives, arrays, object overload)
- Metadata and widget metadata binding
- Writable channels and command invoke handlers
- Local cached reads (`get*` / typed `get`)
- High-rate topic handle usage (`topicDouble/topicI64/...`)
- Layout publishing with ARCP layout DSL
- Typed device widgets via `ArcpDeviceWidgets`
- RobotCore ARCP config and runtime controls

## Entry Files

- `ArcpSignalExamples.java`
- `ArcpControlExamples.java`
- `ArcpReadExamples.java`
- `ArcpLayoutExamples.java`
- `ArcpRobotCoreExamples.java`
