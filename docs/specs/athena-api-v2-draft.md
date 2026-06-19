# Athena API V2 Draft

Status: draft

This document proposes a full replacement for the current Athena config DSL.

The goal is not to incrementally clean up `MechanismConfig`, `SuperstructureConfig`, and
`RobotCoreConfig`. The goal is to replace them with a new API surface that:

- keeps the current nested-lambda style
- removes `Consumer<...>` from user-facing code
- keeps Athena hardware enums like `AthenaMotor` and `AthenaEncoder` in the DSL
- brings motors up to the same flexibility level encoders have gained
- keeps encoders first-class while unifying non-encoder sensing and runtime-fed values
- avoids collision-prone type names that fight WPILib imports
- avoids explicit user-facing generics where they add no value
- splits implementation across many focused files instead of a few thousand-line files
- treats IDE support as a first-class requirement
- does not preserve backward compatibility with the old API

## Decision Summary

V2 should keep `*Config` as the public top-level noun.

- `MechanismConfig` stays `MechanismConfig`
- `StatefulMechanismConfig<State>` should be the stateful root instead of family-specific variants
- `SuperstructureConfig` stays `SuperstructureConfig`
- `RobotCoreConfig` should become `RobotConfig`
- mechanism families like arm/turret/elevator/flywheel should disappear from the public root API

Why keep `Config`:

- it matches how Athena already talks about robot declarations
- it matches how FRC teams already think about these files
- if V1 is being deleted anyway, there is less value in renaming to `Spec` just to create semantic distance
- the real problem is not the word `Config`; the real problem is giant files, nested public builder classes, and poor IDE visibility

V2 should move from nested public builder classes to top-level Athena-specific API types.

- `MotorsSection` becomes `MechanismMotors`
- `EncodersSection` becomes `MechanismEncoders`
- `MechanismEncoderSourceDsl` becomes `MechanismEncoder`
- `SensorsSection` and `InputsSection` collapse into `MechanismInputs`
- `ControlSection`, `ConstraintsSection`, and `HooksSection` live under `MechanismBehavior`
- `MechanismBehavior` contains `control(...)`, `automation(...)`, and `constraints(...)`
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

Reusable section types should also support a static builder/factory form for teams that prefer
object construction over lambda assignment, so both of these remain valid:

- `encoders -> encoders...`
- `MechanismEncoders.create()...`

Because `new` cannot be a Java method name, the API should use something like `create()`,
`builder()`, or `define()`.

Example:

```java
MechanismEncoders encoders = MechanismEncoders.create()
    .add(AthenaEncoder.CANCODER, 40, source -> source.units(Units.Degrees))
    .add("turretCrt", source -> source.crt(crt -> crt.input("40").input("41")));
```

V2 should avoid explicit user-facing generics on reusable section fields wherever possible.

- `MechanismMotors`, `MechanismEncoders`, `MechanismInputs`, and `MechanismSimulation` should be non-generic
- context-bearing config should infer type from the enclosing mechanism config when passed inline
- public behavior config should stay universal; the enclosing `StatefulMechanismConfig<State>` should bind state typing without requiring family-specific public names
- structured behavior should support both a `behavior(...)` method and annotation-driven behavior members/methods like `@ControlLoop(...)` and `@OnStatePeriodic(...)`

V2 should use the rewrite to fix conceptual seams instead of preserving them.

- motors should become composable and named like encoders, rather than a thin append-only list
- non-encoder sensing and external/runtime-fed values should share one public domain
- `hooks` should stop existing as an independent top-level concept
- section composition should be supported directly for reusable config fragments
- superstructures should support stateless children without fake state shims
- hardware-backed registries should use one consistent verb, `add(...)`, with optional explicit keys only when needed

V2 should also drop mechanism families as a first-class authoring concept.

- the public API should not force users to pick `arm`, `turret`, `elevator`, or `flywheel` roots
- anything families still mean today should become explicit declared capabilities
- statefulness should remain real; families should not

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
    Mechanisms.java
    MechanismConfig.java
    StatefulMechanismConfig.java
  mechanism/annotation
    Mechanism.java
  mechanism/annotation/identity
    InitialState.java
    PositionDomain.java
    TravelRange.java
    ContinuousRotation.java
    Feedforward.java
    Visualization.java
    Simulation.java
  mechanism/annotation/input
    DigitalInput.java
    AnalogInput.java
    BooleanInput.java
    DoubleInput.java
  mechanism/annotation/motor
    Motor.java
    Follower.java
    MotorGroup.java
  mechanism/annotation/encoder
    Encoder.java
    DefaultPositionSource.java
    DefaultVelocitySource.java
    EncoderInput.java
  mechanism/annotation/behavior/control
    ControlLoop.java
    LoopSchedule.java
    LoopMode.java
    Pid.java
    Feedforward.java
    BangBang.java
  mechanism/annotation/behavior/state
    OnStateEnter.java
    OnStatePeriodic.java
    OnStateExit.java
  mechanism/annotation/behavior/mode
    OnEnabledEnter.java
    OnEnabledPeriodic.java
    OnEnabledExit.java
    OnTeleEnter.java
    OnTelePeriodic.java
    OnTeleExit.java
    OnAutoEnter.java
    OnAutoPeriodic.java
    OnAutoExit.java
    OnDisabledEnter.java
    OnDisabledPeriodic.java
    OnDisabledExit.java
    OnTestEnter.java
    OnTestPeriodic.java
    OnTestExit.java
  mechanism/identity
    MechanismIdentity.java
    IdentityConfig.java
    MechanismMeasurement.java
    MeasurementConfig.java
    PositionDomain.java
    PositionUnit.java
    FeedforwardProfile.java
    VisualizationProfile.java
    SimulationProfile.java
  mechanism/motor
    MechanismMotors.java
    MechanismMotor.java
    MotorConfig.java
  mechanism/encoder
    MechanismEncoders.java
    EncoderConfig.java
    MechanismEncoder.java
    EncoderSourceConfig.java
  mechanism/input
    MechanismInputs.java
    MechanismDigitalInput.java
    MechanismBooleanInput.java
    MechanismDoubleInput.java
    InputConfig.java
    DigitalInputConfig.java
    AnalogInputConfig.java
    BooleanInputConfig.java
    DoubleInputConfig.java
  mechanism/behavior
    MechanismBehavior.java
    BehaviorConfig.java
  mechanism/behavior/control
    MechanismControl.java
    ControlConfig.java
    MechanismControlLoop.java
    ControlLoopConfig.java
    MechanismPid.java
    PidConfig.java
    MechanismFeedforward.java
    FeedforwardConfig.java
    MechanismBangBang.java
    BangBangConfig.java
  mechanism/behavior/automation
    MechanismAutomation.java
    AutomationConfig.java
  mechanism/behavior/constraint
    MechanismConstraints.java
    ConstraintConfig.java
  mechanism/simulation
    MechanismSimulation.java
    SimulationConfig.java
  superstructure
    SuperstructureConfig.java
    Superstructures.java
  superstructure/annotation
  superstructure/annotation/behavior/state
    OnStateEnter.java
    OnStatePeriodic.java
    OnStateExit.java
  superstructure/annotation/behavior/mode
    OnEnabledEnter.java
    OnEnabledPeriodic.java
    OnEnabledExit.java
  superstructure/part
    SuperstructureParts.java
    PartConfig.java
  superstructure/input
    SuperstructureInputs.java
    InputConfig.java
  superstructure/behavior
    SuperstructureBehavior.java
    BehaviorConfig.java
  superstructure/behavior/automation
    SuperstructureAutomation.java
    AutomationConfig.java
  superstructure/behavior/constraint
    SuperstructureConstraints.java
    ConstraintConfig.java
  robot
    RobotConfig.java
    Robots.java
  robot/annotation
  robot/annotation/mode
    OnEnabledEnter.java
    OnEnabledPeriodic.java
    OnEnabledExit.java
    OnTeleEnter.java
    OnTelePeriodic.java
    OnTeleExit.java
    OnAutoEnter.java
    OnAutoPeriodic.java
    OnAutoExit.java
    OnDisabledEnter.java
    OnDisabledPeriodic.java
    OnDisabledExit.java
    OnTestEnter.java
    OnTestPeriodic.java
    OnTestExit.java
  robot/drivetrain
    RobotDrivetrain.java
  robot/localization
    RobotLocalization.java
  robot/vision
    RobotVision.java
  robot/auto
    RobotAuto.java
  robot/mechanism
    RobotMechanisms.java
  robot/core
    RobotHooks.java
    RobotCore.java
  state
    StateValue.java
    StateFlow.java
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

- root entrypoint types are `MechanismConfig` and `StatefulMechanismConfig<State>`
- declarative products are `*Config`
- lambda target types are Athena-specific nouns: `MechanismMotors`, `MechanismEncoders`, `MechanismInputs`, `MechanismBehavior`
- mutable lambda receivers are `*Config`
- no type name should include `Dsl`
- no type name should include `Section`
- family nouns like `Arm`, `Turret`, `Elevator`, and `Flywheel` should not be used for public root config types
- static mechanism semantics should use a noun like `identity`, not a vague bucket like `capabilities`
- short names that are likely to collide with WPILib should be avoided in the public API

## Three Authoring Styles

V2 should support three authoring styles over one underlying mechanism model.

- `Structured API`: interface/class-based, familiar to Java users, stronger onboarding story
- `Flow API`: compact nested-lambda authoring for advanced users
- `Annotation API`: mostly static declarations via type, field, and method annotations

The structured and annotation styles should be freely mixable in the same file.
The annotation API should be strong enough that many mechanisms can express most of their
identity, inputs, and automation without writing section methods at all.

Both must target the same root model.

Example stateful root:

```java
StatefulMechanismConfig<State>
```

V2 should not have separate implementation stacks for these styles.

## Turret CRT Example

The turret is a good stress test because it wants:

- angular position semantics
- continuous rotation semantics
- multiple encoder sources
- CRT composition
- limit switch homing
- state-driven automation
- control source selection

### Flow Style

```java
import ca.frc6390.athena.api.mechanism.Mechanisms;
import ca.frc6390.athena.api.mechanism.StatefulMechanismConfig;
import ca.frc6390.athena.api.mechanism.identity.PositionDomain;
import ca.frc6390.athena.api.mechanism.identity.VisualizationProfile;
import ca.frc6390.athena.api.mechanism.identity.SimulationProfile;
import ca.frc6390.athena.api.mechanism.encoder.MechanismEncoders;
import ca.frc6390.athena.api.mechanism.input.MechanismInputs;
import ca.frc6390.athena.api.mechanism.motor.MechanismMotors;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismPid;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.mechanisms.OutputType;
import edu.wpi.first.units.Units;

interface Turret {
    MechanismMotors motors = MechanismMotors.create()
        .add(AthenaMotor.KRAKEN_X60, 13, motor -> motor
            .neutralMode(MotorNeutralMode.Brake)
            .canBus("can")
            .currentLimit(10));

    MechanismEncoders absoluteEncoders = MechanismEncoders.create()
        .add(AthenaEncoder.CANCODER, 40, source -> source
            .canBus("can")
            .gearRatio(1.0 / 7.0)
            .units(Units.Degrees)
            .wrapsEvery(360.0 / 7.0))
        .add(AthenaEncoder.CANCODER, 41, source -> source
            .canBus("can")
            .gearRatio(1.0 / 9.0)
            .units(Units.Degrees)
            .wrapsEvery(360.0 / 9.0));

    MechanismEncoders crtEncoders = MechanismEncoders.create()
        .add("turretCrt", source -> source
            .crt(crt -> crt
                .input(40)
                .input(41))
            .units(Units.Degrees));

    MechanismInputs inputs = inputs -> inputs
        .digitalInput("home", digital -> digital
            .dio(6)
            .inverted(true)
            .position(134.0))
        .booleanValue("allowCounterRotation")
        .doubleValue("fieldHeadingDeg");

    MechanismPid mainPid = MechanismPid.create()
        .kp(0.12)
        .ki(0.0)
        .kd(0.0)
        .tolerance(5.0);

    MechanismFeedforward counterRotation = MechanismFeedforward.create()
        .simple()
        .ks(0.33888)
        .kv(0.019484)
        .ka(0.00014993);

    StatefulMechanismConfig<State> config = Mechanisms.create("Turret", State.Off)
        .identity(identity -> identity
            .positionDomain(PositionDomain.ANGULAR, Units.Degrees)
            .travelRange(-134.0, 220.0)
            .measurement(measurement -> measurement
                .position("turretCrt"))
            .visualization(VisualizationProfile.TURRET)
            .simulation(SimulationProfile.SIMPLE_ROTARY))
        .motors(motors)
        .encoders(absoluteEncoders, crtEncoders)
        .inputs(inputs)
        .behavior(behavior -> behavior
            .control(control -> control
                .add("mainPid", loop -> loop
                    .schedule(LoopMode.MANUAL)
                    .output(OutputType.VOLTAGE)
                    .pid(mainPid))
                .add("counterRotation", loop -> loop
                    .schedule(LoopMode.MANUAL)
                    .output(OutputType.VOLTAGE)
                    .feedforward(counterRotation))
                .add("hubTargeting", loop -> loop
                    .schedule(LoopMode.ENABLED)
                    .states(State.Aim)
                    .output(OutputType.VOLTAGE)
                    .custom((robot, control, inputs) -> {
                        double fieldHeading = inputs.doubleValue("fieldHeadingDeg");
                        double target = TurretHelper.get().hubTargetAngle(fieldHeading);
                        double desiredTurretDegPerSec = -robot.imu().yawVelocityZDegreesPerSecond();
                        double desiredNextDegPerSec = desiredTurretDegPerSec
                            + robot.imu().angularAccelerationZDegreesPerSecondSquared() * control.dtSeconds();
                    return control.calculate(mainPid, target)
                        + control.calculate(counterRotation, desiredTurretDegPerSec, desiredNextDegPerSec);
                }))));
}
```

### Structured Style

```java
@Mechanism("Turret")
interface Turret extends StatefulMechanismConfig<State> {
    @Override
    default State initialState() {
        return State.Off;
    }

    @Override
    default void identity(IdentityConfig identity) {
        identity
            .positionDomain(PositionDomain.ANGULAR, Units.Degrees)
            .travelRange(-134.0, 220.0)
            .measurement(measurement -> measurement
                .position("turretCrt"))
            .visualization(VisualizationProfile.TURRET)
            .simulation(SimulationProfile.SIMPLE_ROTARY);
    }

    @Override
    default void motors(MotorConfig motors) {
        motors.add(AthenaMotor.KRAKEN_X60, 13, motor -> motor
            .neutralMode(MotorNeutralMode.Brake)
            .canBus("can")
            .currentLimit(10));
    }

    @Override
    default void encoders(EncoderConfig encoders) {
        encoders.add(AthenaEncoder.CANCODER, 40, source -> source
            .canBus("can")
            .gearRatio(1.0 / 7.0)
            .units(Units.Degrees)
            .wrapsEvery(360.0 / 7.0));

        encoders.add(AthenaEncoder.CANCODER, 41, source -> source
            .canBus("can")
            .gearRatio(1.0 / 9.0)
            .units(Units.Degrees)
            .wrapsEvery(360.0 / 9.0));

        encoders.add("turretCrt", source -> source
            .crt(crt -> crt
                .input(40)
                .input(41))
            .units(Units.Degrees));
    }

    @Override
    default void inputs(InputConfig inputs) {
        inputs.digitalInput("home", digital -> digital
            .dio(6)
            .inverted(true)
            .position(134.0));
        inputs.booleanValue("allowCounterRotation");
        inputs.doubleValue("fieldHeadingDeg");
    }

    @Override
    default void behavior(BehaviorConfig behavior) {
        MechanismPid mainPid = MechanismPid.create()
            .kp(0.12)
            .ki(0.0)
            .kd(0.0)
            .tolerance(5.0);

        MechanismFeedforward counterRotation = MechanismFeedforward.create()
            .simple()
            .ks(0.33888)
            .kv(0.019484)
            .ka(0.00014993);

        behavior.control(control -> control
            .add("mainPid", loop -> loop
                .schedule(LoopMode.MANUAL)
                .output(OutputType.VOLTAGE)
                .pid(mainPid))
            .add("counterRotation", loop -> loop
                .schedule(LoopMode.MANUAL)
                .output(OutputType.VOLTAGE)
                .feedforward(counterRotation))
            .add("hubTargeting", loop -> loop
                .schedule(LoopMode.ENABLED)
                .states(State.Aim)
                .output(OutputType.VOLTAGE)
                .custom((robot, control, inputs) -> {
                    double fieldHeading = inputs.doubleValue("fieldHeadingDeg");
                    double target = TurretHelper.get().hubTargetAngle(fieldHeading);
                    double desiredTurretDegPerSec = -robot.imu().yawVelocityZDegreesPerSecond();
                    double desiredNextDegPerSec = desiredTurretDegPerSec
                        + robot.imu().angularAccelerationZDegreesPerSecondSquared() * control.dtSeconds();
                    return control.calculate(mainPid, target)
                        + control.calculate(counterRotation, desiredTurretDegPerSec, desiredNextDegPerSec);
                })));
    }
}
```

### Annotation Style

```java
@Mechanism("Turret")
@InitialState(State.Off)
@PositionDomain(value = ca.frc6390.athena.api.mechanism.identity.PositionDomain.ANGULAR, units = PositionUnit.DEGREES)
@TravelRange(min = -134.0, max = 220.0)
@Visualization(VisualizationProfile.TURRET)
@Simulation(SimulationProfile.SIMPLE_ROTARY)
interface Turret extends StatefulMechanismConfig<State> {
    @Motor(type = AthenaMotor.KRAKEN_X60, id = 13, bus = "can")
    MechanismMotor turretMotor = MechanismMotor.create()
        .neutralMode(MotorNeutralMode.Brake)
        .currentLimit(10);

    @Encoder(type = AthenaEncoder.CANCODER, id = 40, bus = "can")
    MechanismEncoder absoluteEncoderA = MechanismEncoder.create()
        .gearRatio(1.0 / 7.0)
        .units(Units.Degrees)
        .wrapsEvery(360.0 / 7.0);

    @Encoder(type = AthenaEncoder.CANCODER, id = 41, bus = "can")
    MechanismEncoder absoluteEncoderB = MechanismEncoder.create()
        .gearRatio(1.0 / 9.0)
        .units(Units.Degrees)
        .wrapsEvery(360.0 / 9.0);

    @Encoder
    @DefaultPositionSource
    MechanismEncoder turretCrt = MechanismEncoder.create()
        .crt(crt -> crt
            .input(absoluteEncoderA)
            .input(absoluteEncoderB))
        .units(Units.Degrees);

    @DigitalInput(port = -6)
    MechanismDigitalInput home = MechanismDigitalInput.create()
        .position(134.0);

    @BooleanInput
    MechanismBooleanInput allowCounterRotation = MechanismBooleanInput.create();

    @DoubleInput
    MechanismDoubleInput fieldHeadingDeg = MechanismDoubleInput.create();

    @ControlLoop(output = OutputType.VOLTAGE)
    @LoopSchedule(mode = LoopMode.MANUAL)
    MechanismPid mainPid = MechanismPid.create()
        .kp(0.12)
        .ki(0.0)
        .kd(0.0)
        .tolerance(5.0);

    @ControlLoop(output = OutputType.VOLTAGE)
    @LoopSchedule(mode = LoopMode.MANUAL)
    MechanismFeedforward counterRotation = MechanismFeedforward.create()
        .simple()
        .ks(0.33888)
        .kv(0.019484)
        .ka(0.00014993);

    @ControlLoop(output = OutputType.VOLTAGE)
    @LoopSchedule(mode = LoopMode.ENABLED, states = State.Aim)
    double hubTargetingLoop(RobotCore robot, MechanismControl control, MechanismInputs inputs) {
        double fieldHeading = inputs.doubleValue("fieldHeadingDeg");
        double target = TurretHelper.get().hubTargetAngle(fieldHeading);
        double desiredTurretDegPerSec = -robot.imu().yawVelocityZDegreesPerSecond();
        double desiredNextDegPerSec = desiredTurretDegPerSec
            + robot.imu().angularAccelerationZDegreesPerSecondSquared() * control.dtSeconds();
        return control.calculate(mainPid, target)
            + control.calculate(counterRotation, desiredTurretDegPerSec, desiredNextDegPerSec);
    }

    @OnTeleEnter
    void teleEnter(RobotCore robot);

    @OnTelePeriodic
    void telePeriodic(RobotCore robot, MechanismControl control, MechanismEncoders encoders);

    @OnTeleExit
    void teleExit(RobotCore robot);

    @OnDisabledEnter
    void disabledEnter(RobotCore robot);

    @OnDisabledPeriodic
    void disabledPeriodic(RobotCore robot);

    @OnDisabledExit
    void disabledExit(RobotCore robot);
}
```

Annotation note:

- `@Mechanism("Turret")` is the source-visible structured/annotation baseline
- `@InitialState(...)` is warranted for the annotation API, while `initialState()` remains valid in the structured API
- family identity should come from explicit static identity like `positionDomain(...)`, `travelRange(...)`, `continuousRotation()`, encoder measurement roles, and visualization/simulation/feedforward profiles, not from the root type
- almost all static identity should be annotatable when teams prefer a declaration-first style
- motors and encoders should also support annotation-driven declaration through annotated reusable objects
- device annotations like `@Motor(...)` and `@Encoder(...)` should optionally carry bus metadata for CAN-based hardware
- hardware-backed input annotations like `@DigitalInput(...)` should carry the DIO port directly in the annotation definition
- where Athena already uses signed hardware IDs/ports to express inversion, the annotation API should preserve that shorthand instead of requiring a separate inversion flag
- inputs should support annotation-driven declaration, with hardware-backed inputs like `@DigitalInput` following the motor/encoder pattern more closely than plain value inputs like `@BooleanInput` and `@DoubleInput`
- annotation declarations should default their public key to the field name when no explicit name is supplied, including motors, encoders, inputs, and field-backed loop declarations
- method-backed loop declarations should default the loop name to the method name when no explicit alias is supplied
- shorthand explicit aliases like `@BooleanInput("allowCounterRotation")` or `@Encoder("turretCrt")` should stay supported, but `name = ...` should not be the common path
- type-level repeatable annotations should also be available for the smallest declarations, but Java still requires every annotation to attach to a real declaration
- structured behavior should support annotation-driven authoring for common lifecycle and control cases
- the annotation path should be able to tag a specific encoder directly as the default position/velocity source; if none is tagged Athena may fall back to the first compatible encoder by convention
- measured feedback defaults in the flow/structured DSL can still live under a grouped `measurement(...)` block, but the annotation path should prefer encoder-local role tags over disconnected type-level declarations
- the common composition path should let an enabled custom loop invoke manual PID/feedforward helpers directly and return the final output, instead of forcing a vague loop-to-loop target routing model
- control APIs like `calculate(...)` should accept direct loop declarations/handles in the common path, with string-key lookup reserved for dynamic cases where no static handle exists
- control should support both loop-first and component-first annotation styles, but loop-first should be the primary documented path
- `@ControlLoop(...)` should work directly on `MechanismPid`, `MechanismFeedforward`, `MechanismBangBang`, `MechanismControlLoop`, and method-backed custom loop declarations
- loop lifecycle should be declarable with separate schedule metadata like `@LoopSchedule(...)`, including optional active states, rather than requiring boilerplate `@OnStateEnter` / `@OnStateExit` methods just to toggle a loop
- `@Pid(loop = ...)`, `@Feedforward(loop = ...)`, and `@BangBang(loop = ...)` may still exist as optional secondary annotations for teams that want parity with motor/encoder declaration style
- `MechanismControlLoop` should be reserved for custom or mixed loops; plain PID/FF/bang-bang loops should not need an empty wrapper object
- manual helper loops should not be forced into one shared input shape; a PID helper may consume a target position while a compensation/feedforward helper consumes desired velocity and acceleration
- automatic additive loop composition may still exist, but the primary documented example should be one enabled orchestrator loop calling manual helpers explicitly
- declarative loops should be able to own output mode and schedule directly, while manual helper loops remain callable from custom loops
- `LoopMode` should stay small and explicit, probably `ENABLED`, `TELE`, `AUTO`, `TEST`, `DISABLED`, and `MANUAL`
- `LoopMode.ENABLED` means "run every mechanism tick while enabled"; `MANUAL` means "registered but only stepped/invoked explicitly"
- annotations like `@OnStateEnter(State...)`, `@OnStatePeriodic(State...)`, `@OnStateExit(State...)`, `@OnTeleEnter`, `@OnTelePeriodic`, `@OnTeleExit`, `@OnAutoEnter`, `@OnAutoPeriodic`, `@OnAutoExit`, `@OnDisabledEnter`, `@OnDisabledPeriodic`, `@OnDisabledExit`, `@OnTestEnter`, `@OnTestPeriodic`, `@OnTestExit`, and `@ControlLoop(...)` should be first-class, not editor hacks
- annotated automation methods should support direct parameter injection by type so users can ask for `RobotCore`, `MechanismControl`, `MechanismMotors`, `MechanismEncoders`, and similar runtime objects without being forced through a single `ctx` parameter
- annotation-discovered behavior and `behavior(...)` DSL content should merge into the same underlying model with deterministic ordering
- hardware-backed `motors.add(type, id, ...)` and `encoders.add(type, id, ...)` should default their public key to the ID when no explicit key is provided in the fluent DSL, while annotation declarations default to the field name unless explicitly aliased
- explicit keys should still be supported for derived sources like CRT or for disambiguation
- CRT inputs should infer ratio/modulus/period from the referenced encoder definitions by default; explicit overrides should only exist for ambiguous cases
- `units(Units.Degrees)` is preferred over one-off helpers like `unitDegrees()`

Important properties of this shape:

- encoders stay first-class instead of disappearing into a generic signal bucket
- limit switches and external/runtime-fed values live in the same public `inputs` domain
- multiple encoder fragments can compose directly via `.encoders(sec1, sec2, sec3)`
- all three authoring styles target the same `StatefulMechanismConfig<State>`
- `AthenaMotor` and `AthenaEncoder` stay in the public DSL
- the structured path gives a familiar Java shape without losing the flow path
- the structured and annotation paths should support both reusable fields and automation methods
- motors and encoders both use the same `add(...)` registration pattern instead of `motor(...)` vs `named(...)`
- reusable section objects can be authored either as lambdas or via static builders like `MechanismEncoders.create()`

## System Cleanup Goals

V2 should use the rewrite to make several deeper corrections.

### Families Should Become Identity

V1 still carries arm/turret/elevator/flywheel as mechanism families, but most of the user-facing
authoring surface is already generic. V2 should finish that transition and make family meaning
explicit instead of implicit.

A mechanism should declare identity things like:

- whether it is stateful or stateless
- its control quantity/domain
  example: angular position, linear position, velocity, percent output, custom
- physical travel range when one exists
- whether that quantity is bounded or continuous
- default units for the primary control/feedback quantity
- feedforward profile
  example: arm, elevator, simple, custom
- visualization profile
  example: turret, arm, elevator, rotor, custom
- simulation profile
  example: arm, elevator, flywheel, simple rotary, custom

That makes a "turret" no longer a special root type. It becomes a stateful mechanism that declares
angular position, travel range, turret visualization, and whatever control/simulation
profiles it needs.

This should be named `identity`, not `capabilities`.

- `identity`: stable mechanism facts and defaults
  example: position domain, units, travel range, continuous rotation, visualization profile, simulation profile
- `behavior.constraints`: runtime gates and interlocks
  example: block movement while homing, require an input before entering a state, disallow an output under a certain condition

Static travel range should not live under `constraints`.

### Position Semantics Need To Be Explicit

`positionDomain(...)` is meant to answer one narrow question:

- what kind of quantity is this mechanism primarily controlling and measuring?

Examples:

- `PositionDomain.ANGULAR`
- `PositionDomain.LINEAR`
- `PositionDomain.VELOCITY`
- `PositionDomain.PERCENT`
- `PositionDomain.CUSTOM`

It should define things like:

- default unit family
- what control helpers make sense
- what feedforward/simulation profiles are compatible by default
- whether wrapping is even a meaningful concept

It should not, by itself, say whether the mechanism is bounded or continuous.

Those are separate identity choices:

- `travelRange(min, max)`: bounded physical travel
- `continuousRotation()`: unbounded/wrapping travel

These two should be treated as mutually exclusive and validator-enforced.

`travelRange(...)` should use the canonical unit already established by `positionDomain(...)`.
If `positionDomain(...)` declares an exact unit, a separate `travelRange` unit override should not
exist in the public API because disagreement would only create invalid states.

- bounded turret example: angular + travel range
- continuous turret example: angular + continuous rotation
- elevator example: linear + travel range

The current turret example in this draft uses the bounded form on purpose.

For the annotation path, Athena should use annotation-safe enums like `PositionUnit.DEGREES`
instead of raw WPILib unit objects such as `Units.Degrees`, because Java annotations cannot
carry arbitrary runtime objects.

### Motors Need to Catch Up to Encoders

Motor configuration should become a first-class composable system instead of a mostly linear append API.

Desired capabilities:

- named motors and named groups
- leader/follower relationships
- per-device overrides and per-group defaults
- mixed hardware in one mechanism without awkward branching
- reusable motor fragments combined in order

Target style:

```java
MechanismMotors drivetrainMotors = motors -> motors
    .add(AthenaMotor.KRAKEN_X60, 1, motor -> motor.brake())
    .add("leftFollower", AthenaMotor.KRAKEN_X60, 2, motor -> motor.follow("1"))
    .group("left", "1", "leftFollower");
```

### Encoders Stay Explicit, Inputs Own the Rest

V1 is muddy because `sensors` and `inputs` are really the same domain while `encoders` are special enough to stay visible.

V2 should use these boundaries:

- `encoders`: mechanism feedback sources and encoder-derived math
  example: internal, CANCoder, virtual, CRT, filtered, differentiated, averaged, fused
- `inputs`: everything else the mechanism can read
  example: digital inputs, analog sensors, boolean flags, doubles from another system
- `identity.travelRange(...)`: static physical range when one exists
- `behavior.constraints`: runtime rules that shape what the mechanism is allowed to do
  example: forbidden directions, homing requirements, interlocks

`sensors` should disappear as a public top-level term.

For hardware honesty, limit switches and beam breaks should not be separate first-class device
types in the public API. They are usually just DIO-backed inputs whose meaning comes from the key,
the field name, or the behavior that consumes them.

Hardware-backed inputs should still have reusable object forms for parity with motors and encoders.

Examples:

```java
MechanismDigitalInput home = MechanismDigitalInput.create()...;

MechanismInputs inputs = inputs -> inputs
    .digitalInput("home", home)
    .booleanValue("allowCounterRotation");
```

Simple inputs should also support annotations in the structured path.

Examples:

```java
@DigitalInput(port = -6)
MechanismDigitalInput home = MechanismDigitalInput.create()...;

@BooleanInput
MechanismBooleanInput allowCounterRotation = MechanismBooleanInput.create();
```

The same pattern should extend to motors and encoders.

Examples:

```java
@Motor(type = AthenaMotor.KRAKEN_X60, id = 13, bus = "can")
MechanismMotor turretMotor = MechanismMotor.create()...;

@Encoder(type = AthenaEncoder.CANCODER, id = 40, bus = "can")
MechanismEncoder absoluteEncoderA = MechanismEncoder.create()...;

@Encoder
MechanismEncoder turretCrt = MechanismEncoder.create()...;
```

CRT input binding should accept either stable keys or direct encoder handles.

Examples:

```java
.crt(crt -> crt
    .input("40")
    .input("41"))

.crt(crt -> crt
    .input(absoluteEncoderA)
    .input(absoluteEncoderB))
```

Java does not allow freestanding annotations, so "annotation-only" declarations still have to
attach to something real. The furthest this can go cleanly is:

- type-level repeatable annotations on the mechanism interface/class
- field-level annotated reusable objects
- method-level automation/control annotations

For any input/value teams may want to reference later, annotated reusable fields are still the
better fit than pure type-level annotations.

That is also why keeping `MechanismDigitalInput`, `MechanismBooleanInput`, and similar input
objects is useful even in the annotation API: they provide stable handles that control,
automation, and constraints can refer to later.

The same argument applies to `MechanismMotor` and `MechanismEncoder`: even when they are
declared through annotations, the field still gives the rest of the config graph a stable handle.

Possible type-level-only examples:

```java
@BooleanInput("allowCounterRotation")
@DoubleInput("fieldHeadingDeg")
interface Turret extends StatefulMechanismConfig<State> { ... }
```

CRT-specific note:

- CRT composition should infer ratio/modulus/period from the referenced encoder definitions whenever those definitions provide enough information
- explicit CRT ratio or modulus should be an override, not the normal path

### Behavior Owns Control, Automation, and Constraints

V2 should stop pretending these are separate domains.

- `behavior.control`: output-producing controllers and output calculators
- `behavior.automation`: lifecycle/state/event bindings with side effects
- `behavior.constraints`: runtime rules and interlocks that gate movement or state transitions

That is why this draft prefers one top-level `behavior(...)` section instead of parallel top-level `control(...)` and `hooks(...)` buckets.

Behavior should also support an annotation authoring path for the structured API.

Examples:

```java
@Encoder
@DefaultPositionSource
MechanismEncoder turretCrt = MechanismEncoder.create()...;

@ControlLoop(output = OutputType.VOLTAGE)
@LoopSchedule(mode = LoopMode.MANUAL)
MechanismPid mainPid = MechanismPid.create()...;

@ControlLoop(output = OutputType.VOLTAGE)
@LoopSchedule(mode = LoopMode.MANUAL)
MechanismFeedforward counterRotation = MechanismFeedforward.create()...;

@ControlLoop(output = OutputType.VOLTAGE)
@LoopSchedule(mode = LoopMode.ENABLED, states = State.Aim)
double hubTargetingLoop(RobotCore robot, MechanismControl control, MechanismInputs inputs) { ... }

@OnTelePeriodic
void telePeriodic(RobotCore robot, MechanismControl control, MechanismMotors motors, MechanismEncoders encoders) { ... }
```

These annotations should compile into the same `MechanismBehavior` model as the fluent DSL.
They should not create a separate runtime path with different semantics.

The important split is:

- measured feedback belongs to tagged encoder roles or explicit loop overrides
- PID/feedforward/bang-bang belong to named control loops
- targeting/aiming math can live inside an enabled custom loop that invokes PID and compensation helpers with different inputs and returns the final output

That keeps "hub targeting" from pretending to be a measured position source while also avoiding a
backwards loop-to-loop target relationship in the common case.

Parameter injection should be by supported type, with order irrelevant, so the user can request
only what they need. A single `AutomationContext<State>` parameter can still exist as the escape
hatch, but it should not be required for simple cases.

The annotation surface should be complete and organized by domain, not left as a flat pile.

- `mechanism/annotation/identity/*`: static mechanism declarations
- `mechanism/annotation/input/*`: input declarations
- `mechanism/annotation/motor/*`: motor topology declarations
- `mechanism/annotation/encoder/*`: encoder source declarations
- `mechanism/annotation/behavior/control/*`: controller binding/config annotations
- `mechanism/annotation/behavior/state/*`: `@OnStateEnter`, `@OnStatePeriodic`, `@OnStateExit`
- `mechanism/annotation/behavior/mode/*`: enabled, teleop, auto, disabled, and test enter/periodic/exit

### Section Composition Should Be Built In

Reusable config fragments should combine without custom helper methods.

Examples:

```java
.motors(baseMotors, compBotMotors)
.encoders(coreEncoders, crtEncoders)
.inputs(fieldInputs, homingInputs)
.behavior(baseBehavior, characterizationBehavior)
```

Inside `encoders`, source groups should also compose directly:

```java
config -> config.encoders(primaryEncoders, crtEncoders, simEncoders)
```

Composition order should be deterministic and documented as left-to-right.

### Superstructures Need Stateful and Stateless Parts

V2 should not require every child to be stateful.

Superstructure children should support at least:

- `stateful(...)`: maps a superstructure goal to a child state
- `stateless(...)`: registers and exposes a child with no state mapping
- `managed(...)`: optional future form for a child controlled by a derived goal or callback

Every child should also have a stable key or handle for later lookup, not only a state mapper.

## Superstructure Example

```java
import ca.frc6390.athena.api.superstructure.SuperstructureConfig;
import ca.frc6390.athena.api.superstructure.Superstructures;
import ca.frc6390.athena.api.superstructure.part.SuperstructureParts;

interface TurretAssembly {
    record Goal(Turret.State turret, Hood.State hood, Shooter.State shooter) {}

    SuperstructureParts<State, Goal> parts = parts -> parts
        .stateful("turret", Turret.config, Goal::turret)
        .stateful("hood", Hood.config, Goal::hood)
        .stateful("shooter", Shooter.config, Goal::shooter)
        .stateless("leds", LedStrip.config);

    SuperstructureConfig<State, Goal> config = Superstructures.create("TurretAssembly", State.Stowed)
        .parts(parts)
        .delayMs(40);
}
```

## Robot Example

```java
import ca.frc6390.athena.api.robot.RobotConfig;
import ca.frc6390.athena.api.robot.Robots;
import ca.frc6390.athena.api.robot.drivetrain.RobotDrivetrain;
import ca.frc6390.athena.api.robot.localization.RobotLocalization;
import ca.frc6390.athena.api.robot.mechanism.RobotMechanisms;

interface RobotBase {
    RobotDrivetrain drivetrain = drivetrain -> drivetrain
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
        .drivetrain(drivetrain)
        .localization(localization)
        .mechanisms(mechanisms)
        .auto(auto)
        .hooks(hooks)
        .core(core);
}
```

## State DSL: The Hard Constraint

This is the one place where V2 cannot pretend plain Java and plugin-powered syntax are the same thing.

Exact current syntax:

```java
enum State {
    Stow(s -> s.manualPercent(0.1).until(ctx -> ctx.digitalInput(0)).then(StowPid)),
    StowPid(85.0)
}
```

cannot be plain Java without extra machinery. The current system works by mutating the enum model after parsing.
That is why `javac` can be made to compile it and JDT/VS Code struggles.

V2 should support both of these modes, with one treated as source-visible fallback and one treated as plugin-enhanced syntax:

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
    Stow(StateValue.flow(s -> s.manualPercent(0.1).until(ctx -> ctx.digitalInput(0)).then(StowPid))),
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
- state DSL should support both paths
- `StateValue.flow(...)` should be the source-visible fallback
- exact enum syntax should remain available only when Athena’s compiler and IDE integrations are active

## File Split Plan

The current thousand-line files should be split by responsibility.

`MechanismConfig.java` replacement should be split into:

- `Mechanisms.java`
- `MechanismConfig.java`
- `StatefulMechanismConfig.java`
- one file per public identity/input/motor/encoder/behavior root type
- one file per public sub-builder type
- one file per public behavior annotation type
- one file per identity/profile enum or config type
- no public family-specific root config files
- `mechanism/input/*` replaces the old split between `sensors` and `inputs`
- `mechanism/identity/*` owns explicit family-replacement semantics like domain, travel range, continuity, and default profiles
- `mechanism/behavior/*` owns `control`, `automation`, and `constraints`
- motor, encoder, input, identity, and behavior annotations should live under their own `mechanism/annotation/*` subpackages
- internal apply/build/runtime code under `internal/mechanism`

`SuperstructureConfig.java` replacement should be split into:

- `SuperstructureConfig.java`
- `Superstructures.java`
- `SuperstructureParts.java`
- `SuperstructureInputs.java`
- `SuperstructureBehavior.java`
- `SuperstructureAutomation.java`
- `SuperstructureConstraints.java`
- superstructure annotations should be split by behavior domain, mirroring the mechanism annotation tree
- internal runtime/build code under `internal/superstructure`

`RobotCoreConfig.java` replacement should be split into:

- `RobotConfig.java`
- `Robots.java`
- `RobotDrivetrain.java`
- `RobotLocalization.java`
- `RobotVision.java`
- `RobotAuto.java`
- `RobotMechanisms.java`
- `RobotHooks.java`
- `RobotCore.java`
- robot annotations should be split by mode lifecycle, not left in one flat package
- internal runtime/build code under `internal/robot`

## Import Strategy

V2 imports should be predictable:

- user API lives under `ca.frc6390.athena.api.*`
- hardware enums stay under `ca.frc6390.athena.hardware.*`
- runtime internals are not imported by robot code

Typical robot files should be able to use:

```java
import ca.frc6390.athena.api.mechanism.*;
import ca.frc6390.athena.api.mechanism.identity.*;
import ca.frc6390.athena.api.mechanism.motor.*;
import ca.frc6390.athena.api.mechanism.encoder.*;
import ca.frc6390.athena.api.mechanism.input.*;
import ca.frc6390.athena.api.mechanism.behavior.*;
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
