package ca.frc6390.athena.mechanism.constraint;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertThrows;

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

    @Test
    void clampAppliesOnlyDuringOutputEvaluation() {
        Constraint<Double> clamp = Constraints.clamp(-2.0, 3.0);
        ConstraintContext<Double> context =
                new ConstraintContext<>(0.0, 8.0, MechanismContext.empty());

        assertEquals(8.0, Constraints.evaluate(List.of(clamp), context).value(), 1.0e-9);
        assertEquals(3.0, Constraints.evaluateOutput(List.of(clamp), context).value(), 1.0e-9);
    }

    @Test
    void symmetricClampAndInvalidBoundsAreHandled() {
        Constraint<Double> clamp = Constraints.clamp(4.0);

        assertEquals(-4.0, Constraints.evaluateOutput(
                List.of(clamp),
                new ConstraintContext<>(0.0, -7.0, MechanismContext.empty())).value(), 1.0e-9);
        assertThrows(IllegalArgumentException.class, () -> Constraints.clamp(3.0, -3.0));
        assertThrows(IllegalArgumentException.class, () -> Constraints.clamp(Double.NaN));
    }

    @Test
    void outputConstraintGroupsRetainTheirStage() {
        Constraint<Double> grouped = Constraints.all(
                Constraints.clamp(-5.0, 5.0),
                Constraints.clamp(-2.0, 3.0));
        ConstraintContext<Double> context =
                new ConstraintContext<>(0.0, 8.0, MechanismContext.empty());

        assertEquals(8.0, Constraints.evaluate(List.of(grouped), context).value(), 1.0e-9);
        assertEquals(3.0, Constraints.evaluateOutput(List.of(grouped), context).value(), 1.0e-9);
        assertThrows(IllegalArgumentException.class, () -> Constraints.all(
                Constraints.range(ca.frc6390.athena.hardware.device.Range.of(-1.0, 1.0)),
                Constraints.clamp(1.0)));
    }

    @Test
    void rangeAndMotionDeclarationsExposeStrictestCombinedLimits() {
        List<Constraint<Double>> constraints = List.of(
                Constraints.range(ca.frc6390.athena.hardware.device.Range.of(-5.0, 8.0)),
                Constraints.range(ca.frc6390.athena.hardware.device.Range.of(-2.0, 10.0)),
                Constraints.motion(12.0, 30.0),
                Constraints.motion(9.0, 40.0));

        var range = Constraints.positionRange(constraints);
        var motion = Constraints.motionProfile(constraints);
        assertEquals(-2.0, range.minimum(), 1.0e-9);
        assertEquals(8.0, range.maximum(), 1.0e-9);
        assertEquals(9.0, motion.maxVelocity(), 1.0e-9);
        assertEquals(30.0, motion.maxAcceleration(), 1.0e-9);
    }

}
