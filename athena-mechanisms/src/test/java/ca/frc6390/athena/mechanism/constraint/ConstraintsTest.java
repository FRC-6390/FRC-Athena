package ca.frc6390.athena.mechanism.constraint;

import edu.wpi.first.units.Units;
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
                        Constraints.range(ca.frc6390.athena.hardware.device.Range.of(Units.Value.of(0.0), Units.Value.of(30.0)))),
                new ConstraintContext<>(8.0, 24.0, MechanismContext.empty()));

        assertEquals(0.0, result.value(), 1.0e-9);
    }

    @Test
    void clampAppliesOnlyDuringOutputEvaluation() {
        Constraint<Double> clamp = Constraints.clamp(Units.Volts.of(-2.0), Units.Volts.of(3.0));
        ConstraintContext<Double> context =
                new ConstraintContext<>(0.0, 8.0, MechanismContext.empty());

        assertEquals(8.0, Constraints.evaluate(List.of(clamp), context).value(), 1.0e-9);
        assertEquals(3.0, Constraints.evaluateOutput(List.of(clamp), context).value(), 1.0e-9);
    }

    @Test
    void symmetricClampAndInvalidBoundsAreHandled() {
        Constraint<Double> clamp = Constraints.maxVoltage(Units.Volts.of(4.0));

        assertEquals(-4.0, Constraints.evaluateOutput(
                List.of(clamp),
                new ConstraintContext<>(0.0, -7.0, MechanismContext.empty())).value(), 1.0e-9);
        assertThrows(IllegalArgumentException.class, () -> Constraints.clamp(Units.Volts.of(3.0), Units.Volts.of(-3.0)));
        assertThrows(IllegalArgumentException.class, () -> Constraints.maxVoltage(Units.Volts.of(Double.NaN)));
    }

    @Test
    void outputConstraintGroupsRetainTheirStage() {
        Constraint<Double> grouped = Constraints.all(
                Constraints.clamp(Units.Volts.of(-5.0), Units.Volts.of(5.0)),
                Constraints.clamp(Units.Volts.of(-2.0), Units.Volts.of(3.0)));
        ConstraintContext<Double> context =
                new ConstraintContext<>(0.0, 8.0, MechanismContext.empty());

        assertEquals(8.0, Constraints.evaluate(List.of(grouped), context).value(), 1.0e-9);
        assertEquals(3.0, Constraints.evaluateOutput(List.of(grouped), context).value(), 1.0e-9);
        assertThrows(IllegalArgumentException.class, () -> Constraints.all(
                Constraints.range(ca.frc6390.athena.hardware.device.Range.of(Units.Value.of(-1.0), Units.Value.of(1.0))),
                Constraints.maxVoltage(Units.Volts.of(1.0))));
    }

    @Test
    void rangeAndMotionDeclarationsExposeStrictestCombinedLimits() {
        List<Constraint<Double>> constraints = List.of(
                Constraints.range(ca.frc6390.athena.hardware.device.Range.of(Units.Value.of(-5.0), Units.Value.of(8.0))),
                Constraints.range(ca.frc6390.athena.hardware.device.Range.of(Units.Value.of(-2.0), Units.Value.of(10.0))),
                typedMotion(12.0, 30.0),
                typedMotion(9.0, 40.0));

        var range = Constraints.positionRange(constraints);
        var motion = Constraints.motionProfile(constraints);
        assertEquals(-2.0, range.raw().minimumInUnit(), 1.0e-9);
        assertEquals(8.0, range.raw().maximumInUnit(), 1.0e-9);
        assertEquals(9.0, motion.maxVelocity(), 1.0e-9);
        assertEquals(30.0, motion.maxAcceleration(), 1.0e-9);
    }

    @Test
    void groupedRangeAndMotionDeclarationsExposeTheirMetadata() {
        Constraint<Double> grouped = Constraints.all(
                Constraints.range(ca.frc6390.athena.hardware.device.Range.of(Units.Value.of(-2.0), Units.Value.of(4.0))),
                typedMotion(3.0, 7.0));

        var range = Constraints.positionRange(List.of(grouped));
        var motion = Constraints.motionProfile(List.of(grouped));

        assertEquals(-2.0, range.raw().minimumInUnit(), 1.0e-9);
        assertEquals(4.0, range.raw().maximumInUnit(), 1.0e-9);
        assertEquals(3.0, motion.maxVelocity(), 1.0e-9);
        assertEquals(7.0, motion.maxAcceleration(), 1.0e-9);
    }


    /** Typed motion limit in rotations, reduced to the plain form a rotary binding would store. */
    private static Constraint<Double> typedMotion(double velocity, double acceleration) {
        return ((UnitConstraint) Constraints.motion(Units.RotationsPerSecond.of(velocity),
                Units.RotationsPerSecondPerSecond.of(acceleration))).expressedIn(Units.Rotations);
    }
}
