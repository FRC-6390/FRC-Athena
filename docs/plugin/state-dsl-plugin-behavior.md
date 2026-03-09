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

## VS Code Contract

- `athenaConfigureVscode` should maintain `.vscode/settings.json` for DSL compatibility.
- Required settings include either:
  - preferred mode:
    `java.jdt.ls.javac.enabled = "on"` and `java.completion.engine = "dom"`
    with `java.jdt.ls.java.home` on JDK 24 or newer; or
  - stable fallback mode:
    `java.jdt.ls.javac.enabled = "off"` and `java.completion.engine = "ecj"`
    with `java.jdt.ls.java.home` on JDK 21 or newer.
- In either mode, required settings also include:
  - `java.import.gradle.annotationProcessing.enabled = false`
  - `java.jdt.ls.vmargs` containing required `--add-exports=jdk.compiler/...` entries.
  - `java.import.gradle.java.home` pointing to a Gradle-compatible JDK so project import does not inherit the language-server JDK accidentally.
- Bootstrap should default to the Java 21 / ECJ path.
- `-PathenaVscodeMode=javac` / `ATHENA_VSCODE_MODE=javac` must opt into the javac/DOM path and require JDK 24+ for `java.jdt.ls.java.home`.
- Local Athena plugin development should resolve through `mavenLocal` rather than a
  robot-project `includeBuild(athena-plugin)` override; Buildship can turn that
  composite path into a partial transformed jar that breaks javac plugin discovery.
- IDE annotation processing must stay off for Athena DSL projects; the current
  VS Code Java builder stack can crash while configuring APT and then report false
  enum-constructor/bounds diagnostics instead of the transformed DSL view.

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
