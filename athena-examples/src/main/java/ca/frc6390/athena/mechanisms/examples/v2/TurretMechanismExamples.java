package ca.frc6390.athena.mechanisms.examples.v2;

import ca.frc6390.athena.api.mechanism.MechanismDefinitions;
import ca.frc6390.athena.api.mechanism.TypedStatefulMechanismConfig;
import ca.frc6390.athena.api.mechanism.annotation.Mechanism;
import ca.frc6390.athena.api.mechanism.annotation.behavior.automation.OnStateEnter;
import ca.frc6390.athena.api.mechanism.annotation.behavior.automation.OnStateExit;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.ControlLoop;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopMode;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopSchedule;
import ca.frc6390.athena.api.mechanism.annotation.encoder.DefaultPositionSource;
import ca.frc6390.athena.api.mechanism.annotation.encoder.Encoder;
import ca.frc6390.athena.api.mechanism.annotation.identity.InitialState;
import ca.frc6390.athena.api.mechanism.annotation.identity.PositionDomain;
import ca.frc6390.athena.api.mechanism.annotation.identity.TravelRange;
import ca.frc6390.athena.api.mechanism.annotation.input.BooleanInput;
import ca.frc6390.athena.api.mechanism.annotation.input.DoubleInput;
import ca.frc6390.athena.api.mechanism.annotation.motor.Motor;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismFeedforward;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismLoopContext;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismPid;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.api.mechanism.encoder.MechanismEncoder;
import ca.frc6390.athena.api.mechanism.identity.PositionDomainKind;
import ca.frc6390.athena.api.mechanism.identity.PositionUnit;
import ca.frc6390.athena.api.mechanism.input.MechanismBooleanInput;
import ca.frc6390.athena.api.mechanism.input.MechanismDoubleInput;
import ca.frc6390.athena.api.mechanism.motor.MechanismMotor;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.mechanisms.MechanismContext;
import ca.frc6390.athena.mechanisms.OutputType;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism;

public final class TurretMechanismExamples {
    private TurretMechanismExamples() {
    }

    public enum State implements SetpointProvider<Double> {
        Off(0.0),
        Safe(0.0),
        Aim(0.0);

        private final double setpoint;

        State(double setpoint) {
            this.setpoint = setpoint;
        }

        @Override
        public Double getSetpoint() {
            return setpoint;
        }
    }

    public static MechanismDefinition definition() {
        return MechanismDefinitions.annotation(new AnnotationPrimary());
    }

    @SuppressWarnings("unchecked")
    public static StatefulMechanism<State> build() {
        return (StatefulMechanism<State>) MechanismDefinitions.build(definition());
    }

    @Mechanism("Turret")
    @InitialState("Off")
    @PositionDomain(value = PositionDomainKind.ANGULAR, units = PositionUnit.DEGREES)
    @TravelRange(min = -170.0, max = 170.0)
    public static final class AnnotationPrimary implements TypedStatefulMechanismConfig<State> {
        @Motor(type = AthenaMotor.KRAKEN_X60, id = 50, bus = "can")
        final MechanismMotor turretMotor = MechanismMotor.create();

        @Encoder(type = AthenaEncoder.CANCODER, id = 51, bus = "can")
        @DefaultPositionSource
        final MechanismEncoder turretAbsolute = MechanismEncoder.create();

        @DoubleInput
        final MechanismDoubleInput turretHeadingDeg = MechanismDoubleInput.create()
            .defaultValue(0.0);

        @DoubleInput
        final MechanismDoubleInput fieldHeadingDeg = MechanismDoubleInput.create()
            .defaultValue(0.0);

        @BooleanInput
        final MechanismBooleanInput allowCounterRotation = MechanismBooleanInput.create()
            .defaultValue(true);

        @ControlLoop(output = OutputType.VOLTAGE)
        @LoopSchedule(mode = LoopMode.MANUAL)
        final MechanismPid mainPid = MechanismPid.create()
            .kp(0.12)
            .ki(0.0)
            .kd(0.0)
            .tolerance(1.0);

        @ControlLoop(output = OutputType.VOLTAGE)
        @LoopSchedule(mode = LoopMode.MANUAL)
        final MechanismFeedforward counterRotation = MechanismFeedforward.create()
            .simple()
            .ks(0.0)
            .kv(0.04)
            .ka(0.0);

        @ControlLoop(output = OutputType.VOLTAGE)
        @LoopSchedule(mode = LoopMode.ENABLED, states = {"Aim"})
        double hubTargetingLoop(MechanismLoopContext ctx) {
            double target = ctx.doubleInput("fieldHeadingDeg");
            double measured = ctx.doubleInput("turretHeadingDeg");
            double counterRotationVolts = ctx.input("allowCounterRotation")
                ? ctx.calculate(counterRotation, 0.0, 0.0)
                : 0.0;
            return ctx.calculate(mainPid, measured, target) + counterRotationVolts;
        }

        @OnStateEnter({"Aim"})
        void aimEnter(MechanismContext<?, ?> context) {
            context.enableControlLoop("mainPid");
        }

        @OnStateExit({"Aim"})
        void aimExit(MechanismContext<?, ?> context) {
            context.disableControlLoop("mainPid");
        }

        @Override
        public State initialState() {
            return State.Off;
        }
    }

    /*
    Flow alternative:

    var definition = Mechanisms.stateful("Turret", State.Off)
        .identity(identity -> identity
            .positionDomain(PositionDomainKind.ANGULAR, PositionUnit.DEGREES)
            .travelRange(-170.0, 170.0))
        .motors(motors -> motors.add(AthenaMotor.KRAKEN_X60, 50).bus("can"))
        .encoders(encoders -> encoders.add(
            MechanismEncoder.create("turretAbsolute")
                .type(AthenaEncoder.CANCODER)
                .id(51)
                .bus("can")
                .defaultPositionSource()))
        .inputs(inputs -> inputs
            .doubleInput("turretHeadingDeg", 0.0)
            .doubleInput("fieldHeadingDeg", 0.0)
            .booleanInput("allowCounterRotation", true))
        .behavior(behavior -> behavior
            .control(control -> control
                .add(mainPid)
                .add(counterRotation)
                .customLoop("hubTargetingLoop", loop -> loop
                    .output(OutputType.VOLTAGE)
                    .enabled("Aim")
                    .custom(ctx -> {
                        double counterRotationVolts = ctx.input("allowCounterRotation")
                            ? ctx.calculate(counterRotation, 0.0, 0.0)
                            : 0.0;
                        return ctx.calculate(mainPid, ctx.doubleInput("turretHeadingDeg"), ctx.doubleInput("fieldHeadingDeg"))
                            + counterRotationVolts;
                    })))
            .automation(automation -> automation
                .onStateEnter(ctx -> ctx.enableControlLoop("mainPid"), State.Aim)
                .onStateExit(ctx -> ctx.disableControlLoop("mainPid"), State.Aim)))
        .definition();
    */

    /*
    Structured alternative:

    @Mechanism("Turret")
    interface StructuredTurret extends TypedStatefulMechanismConfig<State> {
        @Override
        default State initialState() {
            return State.Off;
        }

        @Override
        default void identity(IdentityConfig identity) {
            identity.positionDomain(PositionDomainKind.ANGULAR, PositionUnit.DEGREES);
            identity.travelRange(-170.0, 170.0);
        }

        @Override
        default void motors(MechanismMotors motors) {
            motors.add(AthenaMotor.KRAKEN_X60, 50).bus("can");
        }

        @Override
        default void encoders(MechanismEncoders encoders) {
            encoders.add("turretAbsolute", AthenaEncoder.CANCODER, 51).bus("can").defaultPositionSource();
        }

        @Override
        default void inputs(MechanismInputs inputs) {
            inputs.doubleInput("turretHeadingDeg", 0.0);
            inputs.doubleInput("fieldHeadingDeg", 0.0);
            inputs.booleanInput("allowCounterRotation", true);
        }

        @Override
        default void behavior(MechanismBehavior behavior) {
            behavior
                .control(control -> control
                    .add(mainPid)
                    .add(counterRotation)
                    .customLoop("hubTargetingLoop", loop -> loop
                        .output(OutputType.VOLTAGE)
                        .enabled("Aim")
                        .custom(ctx -> {
                            double counterRotationVolts = ctx.input("allowCounterRotation")
                                ? ctx.calculate(counterRotation, 0.0, 0.0)
                                : 0.0;
                            return ctx.calculate(mainPid, ctx.doubleInput("turretHeadingDeg"), ctx.doubleInput("fieldHeadingDeg"))
                                + counterRotationVolts;
                        })))
                .automation(automation -> automation
                    .onStateEnter(ctx -> ctx.enableControlLoop("mainPid"), State.Aim)
                    .onStateExit(ctx -> ctx.disableControlLoop("mainPid"), State.Aim));
        }
    }
    */
}
