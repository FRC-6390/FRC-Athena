package ca.frc6390.athena.mechanisms.examples.v2;

import ca.frc6390.athena.api.mechanism.MechanismDefinitions;
import ca.frc6390.athena.api.mechanism.TypedStatefulMechanismConfig;
import ca.frc6390.athena.api.mechanism.annotation.Mechanism;
import ca.frc6390.athena.api.mechanism.annotation.behavior.control.ControlLoop;
import ca.frc6390.athena.api.mechanism.annotation.identity.InitialState;
import ca.frc6390.athena.api.mechanism.annotation.identity.PositionDomain;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismControlLoop;
import ca.frc6390.athena.api.mechanism.definition.MechanismDefinition;
import ca.frc6390.athena.api.mechanism.identity.PositionDomainKind;
import ca.frc6390.athena.api.mechanism.motor.MechanismMotor;
import ca.frc6390.athena.api.mechanism.annotation.motor.Motor;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.mechanisms.OutputType;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.StatefulMechanism;

public final class SimpleMotorMechanismExamples {
    private SimpleMotorMechanismExamples() {
    }

    public enum State implements SetpointProvider<Double> {
        Off(0.0),
        Intake(0.65),
        Reverse(-0.35);

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

    @Mechanism("SimpleRoller")
    @InitialState("Off")
    @PositionDomain(PositionDomainKind.PERCENT)
    public static final class AnnotationPrimary implements TypedStatefulMechanismConfig<State> {
        @Motor(type = AthenaMotor.KRAKEN_X44, id = 10, bus = "can")
        final MechanismMotor rollerMotor = MechanismMotor.create();

        @ControlLoop(output = OutputType.PERCENT)
        final MechanismControlLoop rollerOutput = MechanismControlLoop.create()
            .custom(ctx -> ctx.setpoint());

        @Override
        public State initialState() {
            return State.Off;
        }
    }

    /*
    Flow alternative:

    var definition = Mechanisms.stateful("SimpleRoller", State.Off)
        .identity(identity -> identity.positionDomain(PositionDomainKind.PERCENT))
        .motors(motors -> motors.add(AthenaMotor.KRAKEN_X44, 10).bus("can"))
        .behavior(behavior -> behavior.control(control -> control
            .customLoop("rollerOutput", loop -> loop
                .output(OutputType.PERCENT)
                .custom(ctx -> ctx.setpoint()))))
        .definition();
    */

    /*
    Structured alternative:

    @Mechanism("SimpleRoller")
    interface StructuredRoller extends TypedStatefulMechanismConfig<State> {
        @Override
        default State initialState() {
            return State.Off;
        }

        @Override
        default void identity(IdentityConfig identity) {
            identity.positionDomain(PositionDomainKind.PERCENT);
        }

        @Override
        default void motors(MechanismMotors motors) {
            motors.add(AthenaMotor.KRAKEN_X44, 10).bus("can");
        }

        @Override
        default void behavior(MechanismBehavior behavior) {
            behavior.control(control -> control.customLoop("rollerOutput", loop -> loop
                .output(OutputType.PERCENT)
                .custom(ctx -> ctx.setpoint())));
        }
    }
    */
}
