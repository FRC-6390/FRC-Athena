package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertNull;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.mechanism.constraint.Constraint;
import ca.frc6390.athena.mechanism.constraint.ConstraintContext;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.constraint.UnitConstraint;
import ca.frc6390.athena.mechanism.interpolation.InterpolationKinds;
import edu.wpi.first.units.Units;
import java.util.List;
import org.junit.jupiter.api.Test;

class TypedControlBindingTest {

    private static MotorDevice motor() {
        return MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
    }

    private static double motionVelocity(Constraint<Double> constraint) {
        return (double) motionComponent(constraint, "profile", "maxVelocity");
    }

    private static double motionAcceleration(Constraint<Double> constraint) {
        return (double) motionComponent(constraint, "profile", "maxAcceleration");
    }

    /** Reads the private motion constraint's profile; the record itself is not part of the API. */
    private static Object motionComponent(Object constraint, String first, String second) {
        try {
            var profileAccessor = constraint.getClass().getDeclaredMethod(first);
            profileAccessor.setAccessible(true);
            Object profile = profileAccessor.invoke(constraint);
            var valueAccessor = profile.getClass().getDeclaredMethod(second);
            valueAccessor.setAccessible(true);
            return valueAccessor.invoke(profile);
        } catch (ReflectiveOperationException exception) {
            throw new AssertionError(exception);
        }
    }

    private static EncoderDevice rotations() {
        return EncoderDevice.of(EncoderKinds.CANCODER, 2).measures(Units.Rotations);
    }

    private static EncoderDevice degrees() {
        return EncoderDevice.of(EncoderKinds.CANCODER, 3).measures(Units.Degrees);
    }

    @Test
    void typedPositionConvertsMeasuresToTheBindingUnitAtDeclaration() {
        var control = Controls.position(motor(), Units.Rotations);

        var fromDegrees = assertInstanceOf(Actions.ControlPosition.class,
                control.position(Units.Degrees.of(90.0)));
        assertEquals(0.25, fromDegrees.position(), 1e-12);

        var fromRotations = assertInstanceOf(Actions.ControlPosition.class,
                control.position(Units.Rotations.of(2.0)));
        assertEquals(2.0, fromRotations.position(), 1e-12);
        assertSame(Units.Rotations, control.unit());
    }

    @Test
    void typedVelocityConvertsRatesAndRejectsPositions() {
        var control = Controls.velocity(motor(), Units.Rotations);

        var fromRpm = assertInstanceOf(Actions.ControlVelocity.class, control.velocity(Units.RPM.of(120.0)));
        assertEquals(2.0, fromRpm.velocity(), 1e-12);

        var fromDegrees = assertInstanceOf(Actions.ControlVelocity.class,
                control.velocity(Units.DegreesPerSecond.of(360.0)));
        assertEquals(1.0, fromDegrees.velocity(), 1e-12);

        assertThrows(IllegalArgumentException.class, () -> control.velocity(Units.Degrees.of(90.0)));
        assertThrows(IllegalStateException.class, () -> control.position(Units.Meters.of(1.0)));
    }

    @Test
    void linearBindingsAcceptDistanceMeasuresAndConvertThem() {
        var control = Controls.position(motor(), Units.Meters);

        var action = assertInstanceOf(Actions.ControlPosition.class, control.position(Units.Inches.of(12.0)));
        assertEquals(0.3048, action.position(), 1e-9);
        assertEquals(MotionKind.LINEAR, control.motionKind());
        assertThrows(IllegalArgumentException.class, () -> control.position(Units.Degrees.of(90.0)));
    }

    @Test
    void dimensionlessBindingsRejectAngleAndDistanceMeasures() {
        var dimensionless = Controls.position(motor(), Units.Value);

        assertEquals(Units.Value, dimensionless.unit());
        assertEquals(MotionKind.UNSPECIFIED, dimensionless.motionKind());
        assertThrows(IllegalArgumentException.class, () -> dimensionless.position(Units.Degrees.of(90.0)));
        assertThrows(IllegalArgumentException.class, () -> dimensionless.continuous(Units.Degrees.of(0), Units.Degrees.of(360)));
    }

    @Test
    void motionKindIsDerivedFromTheUnit() {
        assertEquals(MotionKind.ROTARY, Controls.position(motor(), Units.Degrees).motionKind());
        assertEquals(MotionKind.ROTARY, Controls.velocity(motor(), Units.Rotations).motionKind());
        assertEquals(MotionKind.LINEAR, Controls.velocity(motor(), Units.Inches).motionKind());
    }

    @Test
    void encoderFeedbackMustMeasureTheBindingUnit() {
        var control = Controls.position(motor(), Units.Rotations);

        assertThrows(IllegalArgumentException.class, () -> control.feedback(degrees()));
        assertThrows(IllegalArgumentException.class, () -> control.feedback(
                EncoderDevice.of(EncoderKinds.CANCODER, 4).measures(Units.Meters)));
        assertEquals(Units.Rotations, control.feedback(rotations()).unit());
    }

    @Test
    void encoderFeedbackCheckAppliesToTheVelocityChannel() {
        var control = Controls.velocity(motor(), Units.Rotations);
        assertThrows(IllegalArgumentException.class, () -> control.feedback(degrees()));
    }

    @Test
    void typedRangeConstraintIsConvertedToTheBindingUnit() {
        var control = Controls.position(motor(), Units.Rotations)
                .feedback(rotations())
                .constraint(Constraints.range(Units.Degrees.of(-90.0), Units.Degrees.of(90.0)));

        Range limits = Constraints.positionRange(control.constraints());
        assertEquals(-0.25, limits.raw().minimumInUnit(), 1e-12);
        assertEquals(0.25, limits.raw().maximumInUnit(), 1e-12);
    }

    @Test
    void typedRangeRejectsMixedDimensionsAndInvertedBounds() {
        assertThrows(IllegalArgumentException.class,
                () -> Constraints.range(Units.Degrees.of(0.0), Units.Meters.of(1.0)));
        assertThrows(IllegalArgumentException.class,
                () -> Constraints.range(Units.Degrees.of(90.0), Units.Degrees.of(0.0)));
    }

    @Test
    void typedRangeOfTheWrongDimensionFailsAtDeclaration() {
        var linear = Controls.position(motor(), Units.Meters);
        assertThrows(IllegalArgumentException.class,
                () -> linear.constraint(Constraints.range(Units.Degrees.of(0.0), Units.Degrees.of(10.0))));
    }

    @Test
    void typedMotionConstraintMatchesTheEquivalentPrimitiveLimits() {
        Constraint<Double> typed = Constraints.motion(Units.DegreesPerSecond.of(360.0),
                Units.DegreesPerSecondPerSecond.of(720.0));

        assertInstanceOf(UnitConstraint.class, typed);
        Constraint<Double> rotary = ((UnitConstraint) typed).expressedIn(Units.Rotations);
        assertEquals(1.0, motionVelocity(rotary), 1e-12);
        assertEquals(2.0, motionAcceleration(rotary), 1e-12);
        assertThrows(IllegalArgumentException.class,
                () -> ((UnitConstraint) typed).expressedIn(Units.Meters));

        var control = Controls.position(motor(), Units.Rotations).constraint(typed);
        assertEquals(1.0, motionVelocity(control.constraints().get(0)), 1e-12);
    }

    @Test
    void typedContinuousInputUsesBindingUnit() {
        var control = Controls.position(motor(), Units.Rotations)
                .continuous(Units.Degrees.of(0.0), Units.Degrees.of(360.0));

        assertEquals(0.0, control.continuousRange().raw().minimumInUnit(), 1e-12);
        assertEquals(1.0, control.continuousRange().raw().maximumInUnit(), 1e-12);
    }

    @Test
    void dynamicSupplierWithExplicitUnitIsConvertedPerCycleWithoutMeasures() {
        var control = Controls.position(motor(), Units.Rotations);

        var action = assertInstanceOf(Actions.DynamicControlPosition.class,
                control.position(() -> 45.0, Units.Degrees));
        assertEquals(0.125, action.positionSupplier().getAsDouble(), 1e-12);

        var velocity = assertInstanceOf(Actions.DynamicControlVelocity.class,
                Controls.velocity(motor(), Units.Rotations).velocity(() -> 180.0, Units.Degrees));
        assertEquals(0.5, velocity.velocitySupplier().getAsDouble(), 1e-12);
    }

    @Test
    void typedUntilWithinUsesTheControlUnitForTolerance() {
        var control = Controls.position(motor(), Units.Rotations);

        var wrapped = assertInstanceOf(Actions.WithinTolerance.class,
                control.position(Units.Rotations.of(0.5)).untilWithin(Units.Degrees.of(9.0)));
        assertEquals(0.025, wrapped.tolerance(), 1e-12);

        var velocity = assertInstanceOf(Actions.WithinTolerance.class,
                Controls.velocity(motor(), Units.Rotations).velocity(Units.RotationsPerSecond.of(1.0)).untilWithin(Units.RPM.of(60.0)));
        assertEquals(1.0, velocity.tolerance(), 1e-12);

        assertThrows(IllegalArgumentException.class,
                () -> control.position(Units.Rotations.of(0.5)).untilWithin(Units.Meters.of(1.0)));
    }

    @Test
    void typedInterpolationConvertsInputPointsAndOutputValues() {
        var control = Controls.position(motor(), Units.Rotations);

        var interpolated = control.interpolate(InterpolationKinds.LINEAR, () -> 12.0, Units.Inches)
                .at(Units.Inches.of(0.0), Units.Degrees.of(0.0))
                .at(Units.Inches.of(24.0), Units.Degrees.of(180.0));

        assertEquals(0.25, interpolated.target(), 1e-9);
        assertThrows(IllegalArgumentException.class,
                () -> interpolated.at(Units.Degrees.of(0.0), Units.Degrees.of(0.0)));
    }

    /**
     * A velocity binding takes its range in units per second (the velocity target) and its motion as a velocity
     * clamp and an acceleration slew. A position binding rejects the per-second range.
     */
    @Test
    void velocityBindingsTakeTypedRangeAndMotionInUnitsPerSecond() {
        var shooter = Controls.velocity(motor(), Units.Rotations)
                .constraints(
                        Constraints.range(Units.RotationsPerSecond.of(-200.0), Units.RotationsPerSecond.of(200.0)),
                        Constraints.motion(Units.RotationsPerSecond.of(200.0), Units.RotationsPerSecondPerSecond.of(200.0)),
                        Constraints.maxVoltage(Units.Volts.of(12.0)));

        var high = Constraints.evaluate(shooter.constraints(),
                new ConstraintContext<>(0.0, 300.0, ca.frc6390.athena.mechanism.core.MechanismContext.empty()));
        assertEquals(200.0, high.value(), 1.0e-9);
        var low = Constraints.evaluate(shooter.constraints(),
                new ConstraintContext<>(0.0, -500.0, ca.frc6390.athena.mechanism.core.MechanismContext.empty()));
        assertEquals(-200.0, low.value(), 1.0e-9);

        var profile = Constraints.motionProfile(shooter.constraints());
        assertEquals(200.0, profile.maxVelocity(), 1.0e-9);
        assertEquals(200.0, profile.maxAcceleration(), 1.0e-9);

        assertThrows(IllegalArgumentException.class, () -> Controls.position(motor(), Units.Rotations)
                .constraint(Constraints.range(Units.RotationsPerSecond.of(-200.0), Units.RotationsPerSecond.of(200.0))));
    }

    /** Equivalence: the same physical key gives the same target whether the input is read in inches or meters. */
    @Test
    void interpolationKeyInInchesAndMetersGivesTheSameTarget() {
        double[] inches = {60.0};
        var control = Controls.position(motor(), Units.Rotations);
        var keyedInInches = control.interpolate(InterpolationKinds.LINEAR, () -> inches[0], Units.Inches)
                .at(Units.Inches.of(40.0), Units.Rotations.of(10.0))
                .at(Units.Inches.of(80.0), Units.Rotations.of(20.0));
        assertEquals(15.0, keyedInInches.target(), 1.0e-9);

        double[] meters = {Units.Inches.of(60.0).in(Units.Meters)};
        var keyedInMeters = control.interpolate(InterpolationKinds.LINEAR, () -> meters[0], Units.Meters)
                .at(Units.Inches.of(40.0), Units.Rotations.of(10.0))
                .at(Units.Meters.of(2.032), Units.Rotations.of(20.0));
        assertEquals(keyedInInches.target(), keyedInMeters.target(), 1.0e-9);

        assertThrows(IllegalArgumentException.class,
                () -> control.interpolate(InterpolationKinds.LINEAR, () -> meters[0], Units.Meters)
                        .at(Units.Degrees.of(40.0), Units.Rotations.of(10.0)));
    }

    @Test
    void typedInterpolationPointsNeedAnInputUnit() {
        var untyped = Controls.position(motor(), Units.Rotations).interpolate(InterpolationKinds.LINEAR, () -> 1.0, Units.Value);
        assertThrows(IllegalArgumentException.class,
                () -> untyped.at(Units.Inches.of(0.0), Units.Rotations.of(0.0)));
    }

    @Test
    void mismatchedEncoderUnitsAreReportedWithClearMessage() {
        var error = assertThrows(IllegalArgumentException.class,
                () -> Controls.position(motor(), Units.Rotations).feedback(degrees()));
        assertTrue(error.getMessage().contains("measures"));
    }

    @Test
    void lowLevelWrappersKeepTheBindingUnitAcrossDerivedDeclarations() {
        ControlBinding<?> control = Controls.position(motor(), Units.Degrees)
                .feedback(degrees())
                .disabled(false)
                .slot(1)
                .pid(1.0, 0.0, 0.0);

        assertSame(Units.Degrees, control.unit());
        assertEquals(MotionKind.ROTARY, control.motionKind());
        assertInstanceOf(List.class, control.loops());
    }
}
