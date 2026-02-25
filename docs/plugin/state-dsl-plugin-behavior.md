# Athena State DSL Plugin Behavior

This document defines expected behavior for the Athena Gradle/Javac plugin integration used by the enum state DSL.

## Gradle Plugin Contract

- Applying `ca.frc6390.athena.plugin` must register the `athenaPlugin` extension.
- `athenaPlugin.enabled` gates whether compile-task wiring is applied.
- `athenaPlugin.selfArtifactClasspathEnabled` controls whether the plugin artifact is added to Java compile/processor classpaths.

## Javac Wiring Contract

When enabled, Java compile tasks should receive:

- Compiler plugin flag: `-Xplugin:AthenaStateDsl`
- Required `jdk.compiler` exports via forked JVM args.

## Javac Plugin Contract

- Plugin name is `AthenaStateDsl`.
- Enum annotations in `statespec` are translated during parse phase to inject required DSL/runtime boilerplate.

## Build Script Example

```groovy
plugins {
  id 'java'
  id 'ca.frc6390.athena.plugin'
}

athenaPlugin {
  enabled = true
  selfArtifactClasspathEnabled = true
}
```

## Executable References

- Runtime/plugin classes:
  - `athena-plugin/src/main/java/ca/frc6390/athena/plugin/statespec/AthenaPluginGradlePlugin.java`
  - `athena-plugin/src/main/java/ca/frc6390/athena/plugin/statespec/AthenaPluginExtension.java`
  - `athena-plugin/src/main/java/ca/frc6390/athena/plugin/statespec/AthenaStateDslJavacPlugin.java`
- Plugin-style examples:
  - `athena-examples/src/main/java/ca/frc6390/athena/mechanisms/examples/state/StateDslPluginExamples.java`
- Behavior tests:
  - `athena-test/src/test/java/ca/frc6390/athena/plugin/statespec/AthenaPluginStatespecContractTest.java`
  - `athena-test/src/test/java/ca/frc6390/athena/mechanisms/state/StateDslPluginExamplesTest.java`
