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
import ca.frc6390.athena.api.mechanism.annotation.input.DoubleInput;
import ca.frc6390.athena.api.mechanism.annotation.motor.Motor;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismFeedforward;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismLoopContext;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismPid;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.api.mechanism.encoder.MechanismEncoder;
import ca.frc6390.athena.api.mechanism.identity.PositionDomainKind;
import ca.frc6390.athena.api.mechanism.identity.PositionUnit;
import ca.frc6390.athena.api.mechanism.input.MechanismDoubleInput;
import ca.frc6390.athena.api.mechanism.motor.MechanismMotor;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.mechanisms.OutputType;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism;
import edu.wpi.first.math.util.Units;

public final class FlywheelMechanismExamples {
    private FlywheelMechanismExamples() {
    }

    public enum State implements SetpointProvider<Double> {
        Off(0.0),
        Spinup(rpm(3500.0)),
        Fire(rpm(5200.0));

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

    private static double rpm(double value) {
        return Units.rotationsPerMinuteToRadiansPerSecond(value);
    }

    @Mechanism("ShooterFlywheel")
    @InitialState("Off")
    @PositionDomain(value = PositionDomainKind.VELOCITY, units = PositionUnit.RADIANS)
    public static final class AnnotationPrimary implements TypedStatefulMechanismConfig<State> {
        @Motor(type = AthenaMotor.KRAKEN_X60, id = 20, bus = "can")
        final MechanismMotor shooterMotor = MechanismMotor.create();

        @Encoder(type = AthenaEncoder.CANCODER, id = 21, bus = "can")
        @DefaultPositionSource
        final MechanismEncoder flywheelEncoder = MechanismEncoder.create();

        @DoubleInput
        final MechanismDoubleInput measuredVelocityRadPerSec = MechanismDoubleInput.create()
            .defaultValue(0.0);

        @ControlLoop(output = OutputType.VOLTAGE)
        @LoopSchedule(mode = LoopMode.MANUAL)
        final MechanismPid velocityPid = MechanismPid.create()
            .kp(0.08)
            .ki(0.0)
            .kd(0.0)
            .tolerance(1.0);

        @ControlLoop(output = OutputType.VOLTAGE)
        @LoopSchedule(mode = LoopMode.MANUAL)
        final MechanismFeedforward spinFeedforward = MechanismFeedforward.create()
            .simple()
            .ks(0.18)
            .kv(0.11)
            .ka(0.0);

        @ControlLoop(output = OutputType.VOLTAGE)
        @LoopSchedule(mode = LoopMode.ENABLED, states = {"Spinup", "Fire"})
        double velocityClosedLoop(MechanismLoopContext ctx) {
            double target = ctx.setpoint();
            double measured = ctx.doubleInput("measuredVelocityRadPerSec");
            return ctx.calculate(velocityPid, measured, target)
                + ctx.calculate(spinFeedforward, target);
        }

        @Override
        public State initialState() {
            return State.Off;
        }
    }

    /*
    Flow alternative:

    var definition = Mechanisms.stateful("ShooterFlywheel", State.Off)
        .identity(identity -> identity.positionDomain(PositionDomainKind.VELOCITY, PositionUnit.RADIANS))
        .motors(motors -> motors.add(AthenaMotor.KRAKEN_X60, 20).bus("can"))
        .encoders(encoders -> encoders.add(
            MechanismEncoder.create("flywheelEncoder")
                .type(AthenaEncoder.CANCODER)
                .id(21)
                .bus("can")
                .defaultPositionSource()))
        .inputs(inputs -> inputs.doubleInput("measuredVelocityRadPerSec", 0.0))
        .behavior(behavior -> behavior.control(control -> control
            .add(velocityPid)
            .add(spinFeedforward)
            .customLoop("velocityClosedLoop", loop -> loop
                .output(OutputType.VOLTAGE)
                .enabled("Spinup", "Fire")
                .custom(ctx -> ctx.calculate(velocityPid, ctx.doubleInput("measuredVelocityRadPerSec"), ctx.setpoint())
                    + ctx.calculate(spinFeedforward, ctx.setpoint())))))
        .definition();
    */

    /*
    Structured alternative:

    @Mechanism("ShooterFlywheel")
    interface StructuredFlywheel extends TypedStatefulMechanismConfig<State> {
        @Override
        default State initialState() {
            return State.Off;
        }

        @Override
        default void identity(IdentityConfig identity) {
            identity.positionDomain(PositionDomainKind.VELOCITY, PositionUnit.RADIANS);
        }

        @Override
        default void motors(MechanismMotors motors) {
            motors.add(AthenaMotor.KRAKEN_X60, 20).bus("can");
        }

        @Override
        default void encoders(MechanismEncoders encoders) {
            encoders.add("flywheelEncoder", AthenaEncoder.CANCODER, 21).bus("can").defaultPositionSource();
        }

        @Override
        default void inputs(MechanismInputs inputs) {
            inputs.doubleInput("measuredVelocityRadPerSec", 0.0);
        }

        @Override
        default void behavior(MechanismBehavior behavior) {
            behavior.control(control -> control
                .add(velocityPid)
                .add(spinFeedforward)
                .customLoop("velocityClosedLoop", loop -> loop
                    .output(OutputType.VOLTAGE)
                    .enabled("Spinup", "Fire")
                    .custom(ctx -> ctx.calculate(velocityPid, ctx.doubleInput("measuredVelocityRadPerSec"), ctx.setpoint())
                        + ctx.calculate(spinFeedforward, ctx.setpoint()))));
        }
    }
    */
}
