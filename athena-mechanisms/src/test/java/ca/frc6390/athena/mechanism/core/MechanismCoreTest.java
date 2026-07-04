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
import org.junit.jupiter.api.Test;

class MechanismCoreTest {
    @Test
    void directOutputStatesCarryTheirOutputIntent() {
        MechanismState running = States.percent(0.75);
        MechanismState aimed = States.position(42.0);

        Output.Percent percent = assertInstanceOf(Output.Percent.class, running);
        Output.Position position = assertInstanceOf(Output.Position.class, aimed);

        assertEquals(0.75, percent.percent(), 1.0e-9);
        assertEquals(42.0, position.position(), 1.0e-9);
    }

    @Test
    void childSetStateCarriesCompositeTargetsDirectly() {
        TestMechanism feeder = new TestMechanism(States.neutral());
        TestMechanism kicker = new TestMechanism(States.neutral());
        MechanismState feeding = States.percent(1.0);
        MechanismState kicking = States.velocity(1200.0);

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
        MechanismState stowed = States.position(0.0);
        MechanismState homing = States.percent(0.25)
                .until(ctx -> ctx.timeInStateSeconds() > 0.5)
                .then(stowed);

        States.Conditional conditional = assertInstanceOf(States.Conditional.class, homing);
        assertSame(stowed, conditional.next());
        assertTrue(conditional.condition().test(new MechanismContext(1.0, 0.6, 0.02, true, false, false)));

        States.Sequence sequence = States.sequence()
                .forTime(0.2, States.percent(-0.2))
                .then(stowed);

        assertEquals(1, sequence.steps().size());
        assertSame(stowed, sequence.next());
        assertTrue(sequence.steps().get(0).complete().test(new MechanismContext(1.0, 0.25, 0.02, true, false, false)));
    }

    @Test
    void targetedStatesAndClampsCarryExplicitRefs() {
        MotorRef motor = MotorRef.of(AthenaMotor.SIM, 1);
        EncoderRef encoder = EncoderRef.of(AthenaEncoder.SIM, 2);
        RangeRef range = RangeRef.degrees(-45.0, 90.0);

        MechanismState aimed = States.position(motor, encoder, 120.0).clamp(range);
        States.Clamped clamped = assertInstanceOf(States.Clamped.class, aimed);
        States.TargetedPosition position = assertInstanceOf(States.TargetedPosition.class, clamped.state());

        assertSame(motor, position.motor());
        assertSame(encoder, position.encoder());
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
        private final TestMechanism child = new TestMechanism(States.neutral());
        @InitialState
        public final MechanismState HOME = States.percent(0.2).until(limit::active).then(States.neutral());
    }

    private static final class AnnotatedInitialState implements Mechanism {
        public final MechanismState FIRST = States.neutral();
        @InitialState
        public final MechanismState SECOND = States.percent(0.5);
    }

    private static final class FallbackInitialState implements Mechanism {
        public final MechanismState FIRST = States.neutral();
        public final MechanismState SECOND = States.percent(0.5);
    }
}
