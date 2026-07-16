package ca.frc6390.athena.mechanism.constraint;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;

import ca.frc6390.athena.mechanism.core.MechanismContext;
import java.util.List;
import org.junit.jupiter.api.Test;

class ConstraintsTest {
    @Test
    void overrideCorrectsAnExistingTargetAndRestoresItWhenInactive() {
        boolean[] collisionRisk = {false};
        Constraint<Double> safety = Constraints.override(() -> collisionRisk[0], 2.0);
        ConstraintContext<Double> requested = new ConstraintContext<>(8.0, 24.0, MechanismContext.empty());

        assertEquals(24.0, Constraints.evaluate(List.of(safety), requested).value(), 1.0e-9);
        collisionRisk[0] = true;
        ConstraintResult<Double> corrected = Constraints.evaluate(List.of(safety), requested);
        assertInstanceOf(ConstraintResult.Corrected.class, corrected);
        assertEquals(2.0, corrected.value(), 1.0e-9);
        collisionRisk[0] = false;
        assertEquals(24.0, Constraints.evaluate(List.of(safety), requested).value(), 1.0e-9);
    }

    @Test
    void laterRangeConstraintStillBoundsAnOverrideValue() {
        ConstraintResult<Double> result = Constraints.evaluate(
                List.of(
                        Constraints.override(() -> true, -5.0),
                        Constraints.range(ca.frc6390.athena.hardware.device.Range.of(0.0, 30.0))),
                new ConstraintContext<>(8.0, 24.0, MechanismContext.empty()));

        assertEquals(0.0, result.value(), 1.0e-9);
    }
}
