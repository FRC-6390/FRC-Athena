package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.hardware.ref.DigitalInputRef;
import ca.frc6390.athena.hardware.ref.DigitalInputs;
import ca.frc6390.athena.hardware.ref.EncoderRef;
import ca.frc6390.athena.hardware.ref.MotorRef;
import ca.frc6390.athena.hardware.ref.RangeRef;
import ca.frc6390.athena.hardware.ref.Sim;
import ca.frc6390.athena.hardware.ref.SimRef;
import org.junit.jupiter.api.Test;

class MechanismCoreTest {
    @Test
    void boundOutputStatesCarryTheirOutputIntent() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        EncoderRef encoder = EncoderRef.integrated(motor);
        ControlRef positionControl = Controls.position()
                .motor(motor)
                .feedback(encoder)
                .pid(0.1, 0.0, 0.0);
        MechanismState running = States.percent(motor, 0.75);
        MechanismState aimed = States.position(positionControl, 42.0);

        Output.Percent percent = assertInstanceOf(Output.Percent.class, running);
        Output.Position position = assertInstanceOf(Output.Position.class, aimed);
        States.MotorPercent motorPercent = assertInstanceOf(States.MotorPercent.class, running);
        States.ControlledPosition controlledPosition = assertInstanceOf(States.ControlledPosition.class, aimed);

        assertEquals(0.75, percent.percent(), 1.0e-9);
        assertEquals(42.0, position.position(), 1.0e-9);
        assertSame(motor, motorPercent.motor());
        assertSame(positionControl, controlledPosition.control());
    }

    @Test
    void childSetStateCarriesCompositeTargetsDirectly() {
        TestMechanism feeder = new TestMechanism(States.neutral());
        TestMechanism kicker = new TestMechanism(States.neutral());
        MotorRef feederMotor = MotorRef.of(AthenaMotor.SIM, 1);
        ControlRef kickerVelocity = Controls.velocity()
                .motor(MotorRef.of(AthenaMotor.SIM, 2))
                .feedback(EncoderRef.of(AthenaEncoder.SIM, 3));
        MechanismState feeding = States.percent(feederMotor, 1.0);
        MechanismState kicking = States.velocity(kickerVelocity, 1200.0);

        States.ChildSet set = States.set()
                .set(feeder, feeding)
                .set(kicker, kicking);

        assertEquals(2, set.targets().size());
        assertSame(feeder, set.targets().get(0).mechanism());
        assertSame(feeding, set.targets().get(0).state());
        assertSame(kicker, set.targets().get(1).mechanism());
        assertSame(kicking, set.targets().get(1).state());
    }

    @Test
    void conditionsAndSequencesUseLifecycleContextOnly() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        EncoderRef encoder = EncoderRef.integrated(motor);
        ControlRef positionControl = Controls.position()
                .motor(motor)
                .feedback(encoder);
        MechanismState stowed = States.position(positionControl, 0.0);
        MechanismState homing = States.percent(motor, 0.25)
                .until(ctx -> ctx.timeInStateSeconds() > 0.5)
                .then(stowed);

        States.Conditional conditional = assertInstanceOf(States.Conditional.class, homing);
        assertSame(stowed, conditional.next());
        assertTrue(conditional.condition().test(new MechanismContext(1.0, 0.6, 0.02, true, false, false)));

        States.Sequence sequence = States.sequence()
                .forTime(0.2, States.percent(motor, -0.2))
                .then(stowed);

        assertEquals(1, sequence.steps().size());
        assertSame(stowed, sequence.next());
        assertTrue(sequence.steps().get(0).complete().test(new MechanismContext(1.0, 0.25, 0.02, true, false, false)));
    }

    @Test
    void controlledStatesAndClampsCarryExplicitRefs() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        EncoderRef encoder = EncoderRef.of(AthenaEncoder.SIM, 2);
        RangeRef range = RangeRef.degrees(-45.0, 90.0);
        ControlRef control = Controls.position()
                .motor(motor)
                .feedback(encoder)
                .range(range);

        MechanismState aimed = States.position(control, 120.0).clamp(range);
        States.Clamped clamped = assertInstanceOf(States.Clamped.class, aimed);
        States.ControlledPosition position = assertInstanceOf(States.ControlledPosition.class, clamped.state());

        assertSame(control, position.control());
        assertSame(motor, control.motors().get(0));
        assertSame(encoder, control.feedback().get(0));
        assertSame(range, clamped.range());
    }

    @Test
    void sequenceCanCarryDoOnceActions() {
        boolean[] ran = {false};

        States.Sequence sequence = States.sequence()
                .doOnce(() -> ran[0] = true)
                .then(States.neutral());

        States.DoOnce action = assertInstanceOf(States.DoOnce.class, sequence.steps().get(0).state());
        action.action().run();

        assertTrue(ran[0]);
    }

    @Test
    void introspectorDiscoversRefsStatesAndChildrenByFieldName() {
        ExampleMechanism mechanism = new ExampleMechanism();

        MechanismDefinition definition = MechanismIntrospector.inspect(mechanism);

        assertEquals("exampleMechanism", definition.name());
        assertTrue(definition.children().containsKey("child"));
        assertTrue(definition.states().containsKey("HOME"));
        assertEquals("HOME", definition.initialStateName());
        assertSame(mechanism.HOME, definition.initialState());
        assertTrue(definition.refs().containsKey("motor"));
        assertTrue(definition.refs().containsKey("limit"));
        assertTrue(definition.refs().containsKey("control"));
        assertTrue(definition.refs().containsKey("simulation"));
    }

    @Test
    void initialStateCanBeAnnotatedOrFallsBackToFirstState() {
        AnnotatedInitialState annotated = new AnnotatedInitialState();
        FallbackInitialState fallback = new FallbackInitialState();

        assertSame(annotated.SECOND, annotated.initialState());
        assertSame(fallback.FIRST, fallback.initialState());
    }

    private record TestMechanism(MechanismState initialState) implements Mechanism {
    }

    private static final class ExampleMechanism implements Mechanism {
        private final MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        private final DigitalInputRef limit = DigitalInputs.rio(0);
        private final ControlRef control = Controls.percent()
                .motor(motor)
                .sensor(limit)
                .custom("latched", limit);
        private final SimRef simulation = Sim.motor(motor).moi(0.01);
        private final TestMechanism child = new TestMechanism(States.neutral());
        @InitialState
        public final MechanismState HOME = States.percent(motor, 0.2).until(limit::active).then(States.neutral());
    }

    private static final class AnnotatedInitialState implements Mechanism {
        public final MechanismState FIRST = States.neutral();
        @InitialState
        public final MechanismState SECOND = States.percent(MotorRef.of(AthenaMotor.SIM, 1), 0.5);
    }

    private static final class FallbackInitialState implements Mechanism {
        public final MechanismState FIRST = States.neutral();
        public final MechanismState SECOND = States.percent(MotorRef.of(AthenaMotor.SIM, 1), 0.5);
    }
}
