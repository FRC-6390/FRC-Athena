package ca.frc6390.athena.mechanisms.examples.v2;

import ca.frc6390.athena.api.mechanism.MechanismDefinitions;
import ca.frc6390.athena.api.mechanism.TypedStatefulMechanismConfig;
import ca.frc6390.athena.api.mechanism.annotation.Mechanism;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.ControlLoop;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopMode;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.LoopSchedule;
import ca.frc6390.athena.api.mechanism.annotation.encoder.DefaultPositionSource;
import ca.frc6390.athena.api.mechanism.annotation.encoder.Encoder;
import ca.frc6390.athena.api.mechanism.annotation.identity.InitialState;
import ca.frc6390.athena.api.mechanism.annotation.identity.PositionDomain;
import ca.frc6390.athena.api.mechanism.annotation.identity.TravelRange;
import ca.frc6390.athena.api.mechanism.annotation.input.DigitalInput;
import ca.frc6390.athena.api.mechanism.annotation.input.DoubleInput;
import ca.frc6390.athena.api.mechanism.annotation.motor.Motor;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismFeedforward;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismLoopContext;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismPid;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.api.mechanism.encoder.MechanismEncoder;
import ca.frc6390.athena.api.mechanism.identity.PositionDomainKind;
import ca.frc6390.athena.api.mechanism.identity.PositionUnit;
import ca.frc6390.athena.api.mechanism.input.MechanismDigitalInput;
import ca.frc6390.athena.api.mechanism.input.MechanismDoubleInput;
import ca.frc6390.athena.api.mechanism.motor.MechanismMotor;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.mechanisms.OutputType;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism;

public final class ElevatorMechanismExamples {
    private ElevatorMechanismExamples() {
    }

    public enum State implements SetpointProvider<Double> {
        Home(0.0),
        Mid(0.65),
        High(1.25);

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

    @Mechanism("Elevator")
    @InitialState("Home")
    @PositionDomain(value = PositionDomainKind.LINEAR, units = PositionUnit.METERS)
    @TravelRange(min = 0.0, max = 1.35)
    public static final class AnnotationPrimary implements TypedStatefulMechanismConfig<State> {
        @Motor(type = AthenaMotor.KRAKEN_X60, id = 40, bus = "can")
        final MechanismMotor elevatorMotor = MechanismMotor.create();

        @Encoder(type = AthenaEncoder.CANCODER, id = 41, bus = "can")
        @DefaultPositionSource
        final MechanismEncoder carriageEncoder = MechanismEncoder.create();

        @DigitalInput(port = -5)
        final MechanismDigitalInput bottomHome = MechanismDigitalInput.create();

        @DoubleInput
        final MechanismDoubleInput measuredHeightMeters = MechanismDoubleInput.create()
            .defaultValue(0.0);

        @ControlLoop(output = OutputType.VOLTAGE)
        @LoopSchedule(mode = LoopMode.MANUAL)
        final MechanismPid liftPid = MechanismPid.create()
            .kp(10.0)
            .ki(0.0)
            .kd(0.0)
            .tolerance(0.01);

        @ControlLoop(output = OutputType.VOLTAGE)
        @LoopSchedule(mode = LoopMode.MANUAL)
        final MechanismFeedforward carriageAssist = MechanismFeedforward.create()
            .elevator()
            .ks(0.05)
            .kg(0.24)
            .kv(0.9)
            .ka(0.02);

        @ControlLoop(output = OutputType.VOLTAGE)
        double liftClosedLoop(MechanismLoopContext ctx) {
            return ctx.calculate(liftPid, ctx.doubleInput("measuredHeightMeters"), ctx.setpoint())
                + ctx.calculate(carriageAssist, 0.0, 0.0);
        }

        @Override
        public State initialState() {
            return State.Home;
        }
    }

    /*
    Flow alternative:

    var definition = Mechanisms.stateful("Elevator", State.Home)
        .identity(identity -> identity
            .positionDomain(PositionDomainKind.LINEAR, PositionUnit.METERS)
            .travelRange(0.0, 1.35))
        .motors(motors -> motors.add(AthenaMotor.KRAKEN_X60, 40).bus("can"))
        .encoders(encoders -> encoders.add(
            MechanismEncoder.create("carriageEncoder")
                .type(AthenaEncoder.CANCODER)
                .id(41)
                .bus("can")
                .defaultPositionSource()))
        .inputs(inputs -> inputs
            .digitalInput("bottomHome", -5)
            .doubleInput("measuredHeightMeters", 0.0))
        .behavior(behavior -> behavior.control(control -> control
            .add(liftPid)
            .add(carriageAssist)
            .customLoop("liftClosedLoop", loop -> loop
                .output(OutputType.VOLTAGE)
                .custom(ctx -> ctx.calculate(liftPid, ctx.doubleInput("measuredHeightMeters"), ctx.setpoint())
                    + ctx.calculate(carriageAssist, 0.0, 0.0)))))
        .definition();
    */

    /*
    Structured alternative:

    @Mechanism("Elevator")
    interface StructuredElevator extends TypedStatefulMechanismConfig<State> {
        @Override
        default State initialState() {
            return State.Home;
        }

        @Override
        default void identity(IdentityConfig identity) {
            identity.positionDomain(PositionDomainKind.LINEAR, PositionUnit.METERS);
            identity.travelRange(0.0, 1.35);
        }

        @Override
        default void motors(MechanismMotors motors) {
            motors.add(AthenaMotor.KRAKEN_X60, 40).bus("can");
        }

        @Override
        default void encoders(MechanismEncoders encoders) {
            encoders.add("carriageEncoder", AthenaEncoder.CANCODER, 41).bus("can").defaultPositionSource();
        }

        @Override
        default void inputs(MechanismInputs inputs) {
            inputs.digitalInput("bottomHome", -5);
            inputs.doubleInput("measuredHeightMeters", 0.0);
        }

        @Override
        default void behavior(MechanismBehavior behavior) {
            behavior.control(control -> control
                .add(liftPid)
                .add(carriageAssist)
                .customLoop("liftClosedLoop", loop -> loop
                    .output(OutputType.VOLTAGE)
                    .custom(ctx -> ctx.calculate(liftPid, ctx.doubleInput("measuredHeightMeters"), ctx.setpoint())
                        + ctx.calculate(carriageAssist, 0.0, 0.0))));
        }
    }
    */
}
