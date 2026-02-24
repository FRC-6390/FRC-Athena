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
