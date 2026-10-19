package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

/** Covers the merged {@link Actions.Then} and {@link Actions.Conditional} types. */
class MergedCompositionTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
    private static final ActionCondition NEVER = ctx -> false;

    @Test
    void thenFluentAndStaticFormsProduceTheSameType() {
        Action first = MOTOR.percent(0.2);
        Action second = MOTOR.voltage(Units.Volts.of(3.0));

        Actions.Then fluent = assertInstanceOf(Actions.Then.class, first.then(second));
        Actions.Then fixed = assertInstanceOf(Actions.Then.class, Actions.then(first, second));

        assertSame(first, fluent.action());
        assertSame(second, fluent.next());
        assertSame(first, fixed.action());
        assertSame(second, fixed.next());
    }

    @Test
    void conditionalFluentAndStaticFormsProduceTheSameType() {
        Action body = MOTOR.percent(0.2);
        Action next = MOTOR.neutral();

        Actions.Conditional fluent = assertInstanceOf(Actions.Conditional.class, body.until(NEVER));
        Actions.Conditional fixed = assertInstanceOf(Actions.Conditional.class, Actions.until(NEVER, body));
        assertEquals(fluent, new Actions.Conditional(body, NEVER, null));
        assertEquals(fixed, new Actions.Conditional(body, NEVER, null));

        Actions.Conditional chained = assertInstanceOf(Actions.Conditional.class, body.until(NEVER).then(next));
        Actions.Conditional staticChained = assertInstanceOf(Actions.Conditional.class,
                Actions.then(Actions.until(NEVER, body), next));
        assertSame(next, chained.next());
        assertSame(next, staticChained.next());
    }

    @Test
    void continuationAfterAChainedContinuationSequencesInsteadOfReplacingIt() {
        Action body = MOTOR.percent(0.2);
        Action first = MOTOR.voltage(Units.Volts.of(1.0));
        Action second = MOTOR.neutral();

        Actions.Then chained = assertInstanceOf(Actions.Then.class, body.until(NEVER).then(first).then(second));

        assertInstanceOf(Actions.Conditional.class, chained.action());
        assertSame(second, chained.next());
        Actions.Conditional until = (Actions.Conditional) chained.action();
        assertSame(first, until.next());
    }

    @Test
    void thenKeepsItsSourceAndContinuationWhenTheSourceIsAConditional() {
        Action body = MOTOR.percent(0.2);
        Action next = MOTOR.neutral();

        Actions.Conditional conditional = assertInstanceOf(Actions.Conditional.class,
                Actions.until(NEVER, body).then(next));

        assertSame(body, conditional.action());
        assertSame(next, conditional.next());
    }

    @Test
    void effortLimitedControlSurvivesComposition() {
        ControlBinding<?> position = Controls.position(MOTOR);
        Action limited = position.position(1.0).effort(Units.Amps.of(10)).untilWithin(0.1);

        Actions.WithinTolerance within = assertInstanceOf(Actions.WithinTolerance.class, limited);
        Actions.ControlPosition control = assertInstanceOf(Actions.ControlPosition.class, within.action());
        assertTrue(control.effort().limited());
        assertEquals("WITHIN 0.1", within.describe());
    }
}
