# Athena Plugin

Gradle plugin id: `ca.frc6390.athena.plugin`

Purpose:
- enables the Athena javac plugin (`-Xplugin:AthenaStateDsl`)
- auto-wires plugin artifact on `compileOnly`/`annotationProcessor`
- enables enum DSL boilerplate injection for `@AthenaState`

## Intended Enum Style

```java
@AthenaState
public enum IntakeState {
    STOW(0),
    INTAKE(0.85),
    HOLD,
    UNJAM_ALT(s -> s.manualPercent(-0.35).then(INTAKE)),
    UNJAM;

    @AthenaStateLogic(UNJAM)
    static TransitionDirective<IntakeState> unjam(StateCtx<IntakeState> ctx) {
        if (ctx.timeInState() > 0.25) return TransitionDirective.to(INTAKE);
        return TransitionDirective.stay();
    }
}
```

`@AthenaStateLogic(UNJAM)` is normalized by the compiler plugin to a string-backed annotation value.

## Tuple Setpoint Mode

```java
@AthenaState(IntakeTuple.class)
enum IntakeState {
    Stowed(IntakeArmState.Stow, IntakeRollerState.Off),
    Intaking(IntakeArmState.Out, IntakeRollerState.Intaking),
    Reverse(IntakeArmState.Out, IntakeRollerState.Reversed)
}
```

In this mode, the plugin generates `SetpointProvider<IntakeTuple>` and `getSetpoint()`.

## VS Code IntelliSense

The enum DSL transform is a javac plugin (`-Xplugin:AthenaStateDsl`), so VS Code
must run the Java language server in javac mode with `jdk.compiler` exports.
Current `vscode-java` javac-mode support also requires the language server to
launch on JDK 24 or newer.

When using Athena bootstrap (`athena-bootstrap.gradle`), run:

```bash
./gradlew athenaConfigureVscode
```

This now maintains:
- `java.jdt.ls.javac.enabled = "on"`
- `java.completion.engine = "dom"`
- `java.import.gradle.annotationProcessing.enabled = true`
- `java.jdt.ls.vmargs` with Athena's required `jdk.compiler` exports
- `java.jdt.ls.java.home` set to a detected JDK 24+ when one is available
- `java.import.gradle.java.home` set to a detected Gradle-compatible JDK

`athenaEnableDsl` also applies this automatically.
