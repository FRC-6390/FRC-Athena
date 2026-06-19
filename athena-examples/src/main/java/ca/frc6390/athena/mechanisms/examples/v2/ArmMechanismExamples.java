package ca.frc6390.athena.mechanisms.examples.v2;

import ca.frc6390.athena.api.mechanism.MechanismDefinitions;
import ca.frc6390.athena.api.mechanism.TypedStatefulMechanismConfig;
import ca.frc6390.athena.api.mechanism.annotation.Mechanism;
import ca.frc6390.athena.api.mechanism.annotation.behavior.automation.OnStatePeriodic;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.ControlLoop;
import ca.frc6390.athena.api.mechanism.annotation.encoder.DefaultPositionSource;
import ca.frc6390.athena.api.mechanism.annotation.encoder.Encoder;
import ca.frc6390.athena.api.mechanism.annotation.identity.InitialState;
import ca.frc6390.athena.api.mechanism.annotation.identity.PositionDomain;
import ca.frc6390.athena.api.mechanism.annotation.identity.TravelRange;
import ca.frc6390.athena.api.mechanism.annotation.input.DigitalInput;
import ca.frc6390.athena.api.mechanism.annotation.motor.Motor;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismPid;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.api.mechanism.encoder.MechanismEncoder;
import ca.frc6390.athena.api.mechanism.identity.PositionDomainKind;
import ca.frc6390.athena.api.mechanism.identity.PositionUnit;
import ca.frc6390.athena.api.mechanism.input.MechanismDigitalInput;
import ca.frc6390.athena.api.mechanism.motor.MechanismMotor;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.mechanisms.MechanismContext;
import ca.frc6390.athena.mechanisms.OutputType;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism;

public final class ArmMechanismExamples {
    private ArmMechanismExamples() {
    }

    public enum State implements SetpointProvider<Double> {
        Home(0.0),
        Intake(18.0),
        Score(72.0);

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

    @Mechanism("ArmPivot")
    @InitialState("Home")
    @PositionDomain(value = PositionDomainKind.ANGULAR, units = PositionUnit.DEGREES)
    @TravelRange(min = 0.0, max = 95.0)
    public static final class AnnotationPrimary implements TypedStatefulMechanismConfig<State> {
        @Motor(type = AthenaMotor.KRAKEN_X60, id = 30, bus = "can")
        final MechanismMotor armMotor = MechanismMotor.create();

        @Encoder(type = AthenaEncoder.CANCODER, id = 31, bus = "can")
        @DefaultPositionSource
        final MechanismEncoder armEncoder = MechanismEncoder.create();

        @DigitalInput(port = -4)
        final MechanismDigitalInput homeSwitch = MechanismDigitalInput.create();

        @ControlLoop(output = OutputType.VOLTAGE)
        final MechanismPid holdAngle = MechanismPid.create()
            .kp(0.16)
            .ki(0.0)
            .kd(0.0)
            .tolerance(1.0);

        @OnStatePeriodic({"Home"})
        void zeroAtHome(MechanismContext<?, ?> context) {
            if (context.input("homeSwitch")) {
                context.mechanism().setpoint(0.0);
            }
        }

        @Override
        public State initialState() {
            return State.Home;
        }
    }

    /*
    Flow alternative:

    var definition = Mechanisms.stateful("ArmPivot", State.Home)
        .identity(identity -> identity
            .positionDomain(PositionDomainKind.ANGULAR, PositionUnit.DEGREES)
            .travelRange(0.0, 95.0))
        .motors(motors -> motors.add(AthenaMotor.KRAKEN_X60, 30).bus("can"))
        .encoders(encoders -> encoders.add(
            MechanismEncoder.create("armEncoder")
                .type(AthenaEncoder.CANCODER)
                .id(31)
                .bus("can")
                .defaultPositionSource()))
        .inputs(inputs -> inputs.digitalInput("homeSwitch", -4))
        .behavior(behavior -> behavior
            .control(control -> control.pid("holdAngle", pid -> pid
                .output(OutputType.VOLTAGE)
                .kp(0.16)
                .ki(0.0)
                .kd(0.0)))
            .automation(automation -> automation.onStatePeriodic(ctx -> {
                if (ctx.input("homeSwitch")) {
                    ctx.mechanism().setpoint(0.0);
                }
            }, State.Home)))
        .definition();
    */

    /*
    Structured alternative:

    @Mechanism("ArmPivot")
    interface StructuredArm extends TypedStatefulMechanismConfig<State> {
        @Override
        default State initialState() {
            return State.Home;
        }

        @Override
        default void identity(IdentityConfig identity) {
            identity.positionDomain(PositionDomainKind.ANGULAR, PositionUnit.DEGREES);
            identity.travelRange(0.0, 95.0);
        }

        @Override
        default void motors(MechanismMotors motors) {
            motors.add(AthenaMotor.KRAKEN_X60, 30).bus("can");
        }

        @Override
        default void encoders(MechanismEncoders encoders) {
            encoders.add("armEncoder", AthenaEncoder.CANCODER, 31).bus("can").defaultPositionSource();
        }

        @Override
        default void inputs(MechanismInputs inputs) {
            inputs.digitalInput("homeSwitch", -4);
        }

        @Override
        default void behavior(MechanismBehavior behavior) {
            behavior
                .control(control -> control.pid("holdAngle", pid -> pid
                    .output(OutputType.VOLTAGE)
                    .kp(0.16)
                    .ki(0.0)
                    .kd(0.0)))
                .automation(automation -> automation.onStatePeriodic(ctx -> {
                    if (ctx.input("homeSwitch")) {
                        ctx.mechanism().setpoint(0.0);
                    }
                }, State.Home));
        }
    }
    */
}
