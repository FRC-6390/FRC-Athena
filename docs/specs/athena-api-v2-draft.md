# Athena API V2 Draft

Status: draft

This document proposes a full replacement for the current Athena config DSL.

The goal is not to incrementally clean up `MechanismConfig`, `SuperstructureConfig`, and
`RobotCoreConfig`. The goal is to replace them with a new API surface that:

- keeps the current nested-lambda style
- removes `Consumer<...>` from user-facing code
- keeps Athena hardware enums like `AthenaMotor` and `AthenaEncoder` in the DSL
- avoids collision-prone type names that fight WPILib imports
- avoids explicit user-facing generics where they add no value
- splits implementation across many focused files instead of a few thousand-line files
- treats IDE support as a first-class requirement
- does not preserve backward compatibility with the old API

## Decision Summary

V2 should keep `*Config` as the public top-level noun.

- `MechanismConfig` stays `MechanismConfig`
- `SuperstructureConfig` stays `SuperstructureConfig`
- `RobotCoreConfig` should become `RobotConfig`
- family-specific configs should be allowed: `ArmConfig`, `ElevatorConfig`, `FlywheelConfig`, `TurretConfig`

Why keep `Config`:

- it matches how Athena already talks about robot declarations
- it matches how FRC teams already think about these files
- if V1 is being deleted anyway, there is less value in renaming to `Spec` just to create semantic distance
- the real problem is not the word `Config`; the real problem is giant files, nested public builder classes, and poor IDE visibility

V2 should move from nested public builder classes to top-level Athena-specific API types.

- `MotorsSection` becomes `MechanismMotors`
- `EncodersSection` becomes `MechanismEncoders`
- `MechanismEncoderSourceDsl` becomes `MechanismEncoderSource`
- `SensorsSection` becomes `MechanismSensors`
- `ControlSection` becomes `MechanismControl`
- `ConstraintsSection` becomes `MechanismLimits`
- `HooksSection` becomes `MechanismHooks`
- `InputsSection` becomes `MechanismInputs`
- `SimSection` becomes `MechanismSimulation`
- `MechanismsSection` in superstructures becomes `SuperstructureParts`

The Athena-specific prefixes are intentional.

- `Motors`, `Encoders`, and `Sensors` are too collision-prone in FRC codebases
- WPILib already owns a lot of short hardware vocabulary
- the Athena public API should be distinct on import, not only distinct by package path

V2 should replace `Consumer<Section>` parameters with named functional interfaces.

Example:

```java
public interface MechanismMotors {
    MotorConfig apply(MotorConfig motors);
}
```

User code stays lambda-based, but it no longer has to import `Consumer`.

V2 should avoid explicit user-facing generics on reusable section fields wherever possible.

- `MechanismMotors`, `MechanismEncoders`, `MechanismSensors`, `MechanismInputs`, `MechanismLimits`, and `MechanismSimulation` should be non-generic
- context-bearing config should infer type from the family config builder when passed inline
- if a reusable typed callback object is needed, Athena should prefer family-specific names like `ArmHooks<State>` over `Hooks<ArmMechanism<State>>`

## Non-Negotiable Rules

1. No public user-facing nested classes.
2. No public user-facing API file should be larger than roughly 300-400 lines.
3. User-facing types and runtime/internal application code must live in separate packages.
4. All package paths must match declared Java packages exactly.
5. IDE support is a first-class requirement, not an afterthought.
6. V2 replaces V1. There is no compatibility layer requirement.

## Proposed Package Layout

```text
ca.frc6390.athena.api
  mechanism
    MechanismConfig.java
    ArmConfig.java
    ElevatorConfig.java
    FlywheelConfig.java
    TurretConfig.java
    Arms.java
    Elevators.java
    Flywheels.java
    Turrets.java
    GenericMechanisms.java
  mechanism/section
    MechanismMotors.java
    MotorConfig.java
    MechanismEncoders.java
    EncoderConfig.java
    MechanismEncoderSource.java
    EncoderSourceConfig.java
    MechanismSensors.java
    SensorConfig.java
    MechanismLimitSwitch.java
    LimitSwitchConfig.java
    MechanismControl.java
    ControlConfig.java
    MechanismPid.java
    PidConfig.java
    MechanismFeedforward.java
    FeedforwardConfig.java
    MechanismBangBang.java
    BangBangConfig.java
    MechanismInputs.java
    InputConfig.java
    MechanismHooks.java
    HookConfig.java
    MechanismLimits.java
    LimitConfig.java
    MechanismSimulation.java
    SimulationConfig.java
  superstructure
    SuperstructureConfig.java
    Superstructures.java
  superstructure/section
    SuperstructureParts.java
    PartConfig.java
    SuperstructureHooks.java
    HookConfig.java
    SuperstructureLimits.java
    LimitConfig.java
    SuperstructureInputs.java
    InputConfig.java
  robot
    RobotConfig.java
    Robots.java
  robot/section
    RobotDrive.java
    RobotLocalization.java
    RobotVision.java
    RobotAuto.java
    RobotMechanisms.java
    RobotHooks.java
    RobotCore.java
  state
    StateValue.java
    StateMachineDsl.java
    StateMachineConfig.java
    StateHooks.java
```

Internal mutable builders, runtime glue, JSON/TOML application, and factory code should move under:

```text
ca.frc6390.athena.internal
```

That package is not part of the stable user API.

## User-Facing Naming

V2 naming should follow these rules:

- entrypoint types are nouns: `Arms`, `Turrets`, `Robots`, `Superstructures`
- declarative products are `*Config`
- lambda target types are Athena-specific nouns: `MechanismMotors`, `MechanismEncoders`, `MechanismControl`
- mutable lambda receivers are `*Config`
- no type name should include `Dsl`
- no type name should include `Section`
- short names that are likely to collide with WPILib should be avoided in the public API

## Mechanism Example

This is the target style for a mechanism definition.

```java
import ca.frc6390.athena.api.mechanism.ArmConfig;
import ca.frc6390.athena.api.mechanism.Arms;
import ca.frc6390.athena.api.mechanism.section.MechanismEncoderSource;
import ca.frc6390.athena.api.mechanism.section.MechanismEncoders;
import ca.frc6390.athena.api.mechanism.section.MechanismLimitSwitch;
import ca.frc6390.athena.api.mechanism.section.MechanismMotors;
import ca.frc6390.athena.api.mechanism.section.MechanismPid;
import ca.frc6390.athena.api.mechanism.section.MechanismSensors;
import ca.frc6390.athena.api.mechanism.section.MechanismSimulation;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.mechanisms.OutputType;

interface Hood {
    MechanismMotors motors = motors -> motors
        .add(AthenaMotor.KRAKEN_X60, -44)
        .neutralMode(MotorNeutralMode.Brake)
        .canBus("can")
        .currentLimit(20);

    MechanismEncoderSource mainEncoder = source -> source
        .encoder(AthenaEncoder.CANCODER, 45)
        .gearRatio(0.06578947)
        .unitDegrees()
        .conversion(360)
        .offset(85)
        .wrapsEvery(0.06578947 * 360.0);

    MechanismEncoders encoders = encoders -> encoders
        .main(mainEncoder);

    MechanismLimitSwitch homeLimit = limit -> limit
        .dio(9)
        .inverted(true)
        .position(85.0)
        .hardStopPositive()
        .delaySeconds(0.0);

    MechanismSensors sensors = sensors -> sensors
        .limitSwitch("HoodLimit", homeLimit);

    MechanismPid mainPid = pid -> pid
        .output(OutputType.PERCENT)
        .kp(0.0333)
        .ki(0.0)
        .kd(0.0)
        .tolerance(5.0);

    MechanismSimulation simulation = sim -> sim
        .arm(arm -> arm
            .lengthMeters(LENGTH_METERS)
            .reduction(GEAR_RATIO)
            .degrees(0.0, 70.0)
            .startingDegrees(85.0)
            .gravity(false));

    ArmConfig<State> config = Arms.create("Hood", State.Stow)
        .motors(motors)
        .encoders(encoders)
        .sensors(sensors)
        .control(control -> control
            .pid("main", mainPid)
            .periodic("main"))
        .hooks(hooks -> hooks
            .onStatePeriodic(ctx -> {
                if (TurretHelper.get().getShot() != null) {
                    ctx.mechanism().control().setpoint(TurretHelper.get().getShot().getAngle());
                }
            }, State.Aim))
        .simulation(simulation);
}
```

Important properties of this shape:

- nested lambdas still exist
- each reusable lambda target has a dedicated Athena type
- no `Consumer<...>` shows up in user code
- no giant `MechanismConfig.*Section` imports
- `AthenaMotor` and `AthenaEncoder` stay in the public DSL
- user code does not need `Motors<ArmMechanism<State>>`-style generic declarations for the common reusable sections

## Superstructure Example

```java
import ca.frc6390.athena.api.superstructure.SuperstructureConfig;
import ca.frc6390.athena.api.superstructure.Superstructures;
import ca.frc6390.athena.api.superstructure.section.SuperstructureParts;

interface TurretAssembly {
    record Goal(Turret.State turret, Hood.State hood, Shooter.State shooter) {}

    SuperstructureParts<State, Goal> parts = parts -> parts
        .mechanism(Turret.config, Goal::turret)
        .mechanism(Hood.config, Goal::hood)
        .mechanism(Shooter.config, Goal::shooter);

    SuperstructureConfig<State, Goal> config = Superstructures.create("TurretAssembly", State.Stowed)
        .parts(parts)
        .delayMs(40);
}
```

## Robot Example

```java
import ca.frc6390.athena.api.robot.RobotConfig;
import ca.frc6390.athena.api.robot.Robots;
import ca.frc6390.athena.api.robot.section.RobotDrive;
import ca.frc6390.athena.api.robot.section.RobotLocalization;
import ca.frc6390.athena.api.robot.section.RobotMechanisms;

interface RobotBase {
    RobotDrive drive = drive -> drive
        .swerve(swerve -> swerve
            .trackWidthMeters(TRACKWIDTH_METERS, WHEELBASE_METERS)
            .imuPigeon2(false)
            .canBus(canivore)
            .modules(modules));

    RobotLocalization localization = localization -> localization
        .pose("field", pose -> pose
            .odometry()
            .backend(backend -> backend.visionDisabled()));

    RobotMechanisms mechanisms = mechanisms -> mechanisms
        .superstructure(RobotSuperstructure.config);

    RobotConfig config = Robots.create("Rebuilt-2027")
        .drive(drive)
        .localization(localization)
        .mechanisms(mechanisms)
        .auto(auto)
        .hooks(hooks)
        .core(core);
}
```

## State DSL: The Hard Constraint

This is the one place where V2 has to make a hard decision.

Exact current syntax:

```java
enum State {
    Stow(s -> s.manualPercent(0.1).until(ctx -> ctx.limitSwitch(0)).then(StowPid)),
    StowPid(85.0)
}
```

cannot be plain Java without extra machinery. The current system works by mutating the enum model after parsing.
That is why `javac` can be made to compile it and JDT/VS Code struggles.

V2 must choose one of these options:

### Option A: Keep Exact Enum Syntax

Requirements:

- Athena owns a real `javac` plugin
- Athena owns a real JDT/Language Server integration
- Athena owns generated source stubs or synthetic constructor support for IDEs

Pros:

- preserves exact current state enum syntax

Cons:

- the IDE integration becomes a permanent product surface, not a sidecar hack
- IntelliSense quality will always depend on Athena maintaining compiler and editor integrations

### Option B: Keep Lambda-Based States, Change the Wrapper

Example:

```java
enum State implements DoubleState<State> {
    Stow(StateValue.dsl(s -> s.manualPercent(0.1).until(ctx -> ctx.limitSwitch(0)).then(StowPid))),
    StowPid(StateValue.point(85.0));

    private final StateValue<Double, State> value;

    State(StateValue<Double, State> value) {
        this.value = value;
    }

    @Override
    public StateValue<Double, State> value() {
        return value;
    }
}
```

Pros:

- source-visible
- JDT-friendly
- no compiler mutation required

Cons:

- not the exact current enum constant syntax

Recommendation:

- mechanism, superstructure, and robot DSL should become pure source-visible Java
- state DSL should move to Option B unless Athena is willing to treat JDT integration as a first-class maintained product

## File Split Plan

The current thousand-line files should be split by responsibility.

`MechanismConfig.java` replacement should be split into:

- `MechanismConfig.java`
- family configs like `ArmConfig.java` and `TurretConfig.java`
- `Arms.java`, `Elevators.java`, `Flywheels.java`, `Turrets.java`
- one file per public section type
- one file per public sub-builder type
- internal apply/build/runtime code under `internal/mechanism`

`SuperstructureConfig.java` replacement should be split into:

- `SuperstructureConfig.java`
- `Superstructures.java`
- `SuperstructureParts.java`
- `SuperstructureHooks.java`
- `SuperstructureInputs.java`
- `SuperstructureLimits.java`
- internal runtime/build code under `internal/superstructure`

`RobotCoreConfig.java` replacement should be split into:

- `RobotConfig.java`
- `Robots.java`
- `RobotDrive.java`
- `RobotLocalization.java`
- `RobotVision.java`
- `RobotAuto.java`
- `RobotMechanisms.java`
- `RobotHooks.java`
- `RobotCore.java`
- internal runtime/build code under `internal/robot`

## Import Strategy

V2 imports should be predictable:

- user API lives under `ca.frc6390.athena.api.*`
- hardware enums stay under `ca.frc6390.athena.hardware.*`
- runtime internals are not imported by robot code

Typical robot files should be able to use:

```java
import ca.frc6390.athena.api.mechanism.*;
import ca.frc6390.athena.api.mechanism.section.*;
import ca.frc6390.athena.hardware.encoder.*;
import ca.frc6390.athena.hardware.motor.*;
```

without pulling nested implementation classes into scope and without colliding with common WPILib names.

## Build and IDE Support Requirements

V2 should ship with:

- package-correct source jars
- no public API type whose source file path disagrees with its package
- no IDE dependence on nested generic section members
- no user-facing API that requires `Consumer<...>` imports
- JDT support for any syntax Athena expects users to type

If V2 keeps exact enum constant state syntax, the JDT integration is not optional.

## Removal Plan

V1 removal is allowed.

Recommended sequence:

1. build V2 in `ca.frc6390.athena.api`
2. port Athena examples to V2 only
3. port the robot projects to V2
4. delete V1 public builder/config APIs
5. delete V1-specific IDE workarounds

There is no need to preserve V1 binary or source compatibility if the project is willing to make this a clean break.
