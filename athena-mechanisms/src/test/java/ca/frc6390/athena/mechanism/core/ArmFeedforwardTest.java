package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.mechanism.control.FeedforwardGains;
import ca.frc6390.athena.mechanism.model.Identification;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.Units;
import java.util.List;
import org.junit.jupiter.api.Test;

class ArmFeedforwardTest {
    private static final double KG = 2.0;
    private static final double TOLERANCE = 1e-9;

    private static MotorDevice motor() {
        return MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
    }

    /** Context at a fixed position with a stationary target, so only the gravity term is non-zero. */
    private static ControlLoopContext at(double position) {
        return new ControlLoopContext() {
            @Override
            public Output request() {
                return Outputs.position(position);
            }

            @Override
            public double position() {
                return position;
            }
        };
    }

    /** Gravity voltage a bound arm loop produces at a position (kS and kV are zero in these tests). */
    private static double gravity(FeedforwardGains gains, double position) {
        ControlOutput output = gains.bind(null).calculate(at(position));
        return assertInstanceOf(ControlOutput.Voltage.class, output).value();
    }

    @Test
    void armGravityFollowsCosineOfTheAngleInDegrees() {
        FeedforwardGains arm = FeedforwardGains.arm(0.0, 0.0, KG).withGravityAngle(Units.Degrees);
        assertEquals(KG, gravity(arm, 0.0), TOLERANCE);
        assertEquals(KG * Math.cos(Math.toRadians(45.0)), gravity(arm, 45.0), TOLERANCE);
        assertEquals(0.0, gravity(arm, 90.0), TOLERANCE);
        assertEquals(-KG, gravity(arm, 180.0), TOLERANCE);
    }

    @Test
    void gravityOffsetIsAddedToTheAngleInRadians() {
        FeedforwardGains arm = FeedforwardGains.arm(0.0, 0.0, KG, Units.Radians.of(Math.PI / 2.0)).withGravityAngle(Units.Degrees);
        // cos(0 + pi/2) = 0: the offset moves the zero of the gravity term.
        assertEquals(0.0, gravity(arm, 0.0), TOLERANCE);
        assertEquals(-KG, gravity(arm, 90.0), TOLERANCE);
    }

    /** Equivalence: an arm offset given in degrees is the same gravity term as the same offset in radians. */
    @Test
    void gravityOffsetInDegreesMatchesTheSameOffsetInRadians() {
        FeedforwardGains degrees = FeedforwardGains.arm(0.0, 0.0, KG, Units.Degrees.of(30.0)).withGravityAngle(Units.Degrees);
        FeedforwardGains radians = FeedforwardGains.arm(0.0, 0.0, KG, Units.Radians.of(Math.toRadians(30.0)))
                .withGravityAngle(Units.Degrees);

        assertEquals(Math.toRadians(30.0), degrees.gravityOffset().in(Units.Radians), 1.0e-12);
        assertEquals(radians.gravityOffset().in(Units.Radians), degrees.gravityOffset().in(Units.Radians), 1.0e-12);
        for (double angle : new double[] {-90.0, 0.0, 15.0, 90.0, 200.0}) {
            assertEquals(gravity(radians, angle), gravity(degrees, angle), 1.0e-12);
        }
    }

    /** Equivalence: a typed offset on a binding gives the same arm feedforward as the same offset in radians. */
    @Test
    void bindingArmOffsetKeepsTheNumbersOfTheRadianForm() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 11).measures(Units.Degrees);
        FeedforwardGains typed = feedforward(Controls.position(motor(), Units.Degrees)
                .feedback(encoder)
                .armFf(0.0, 0.0, KG, Units.Degrees.of(-12.5)));
        FeedforwardGains plain = feedforward(Controls.position(motor(), Units.Degrees)
                .feedback(encoder)
                .armFf(0.0, 0.0, KG, Units.Radians.of(Math.toRadians(-12.5))));

        assertEquals(plain.gravityOffset().in(Units.Radians), typed.gravityOffset().in(Units.Radians), 1.0e-12);
        assertEquals(gravity(plain, 40.0), gravity(typed, 40.0), 1.0e-12);
    }

    @Test
    void rotationsAndRadiansUseTheirOwnAngleUnits() {
        FeedforwardGains rotations = FeedforwardGains.arm(0.0, 0.0, KG).withGravityAngle(Units.Rotations);
        assertEquals(KG * Math.cos(2.0 * Math.PI * 0.125), gravity(rotations, 0.125), TOLERANCE);
        FeedforwardGains radians = FeedforwardGains.arm(0.0, 0.0, KG).withGravityAngle(Units.Radians);
        assertEquals(KG * Math.cos(0.7), gravity(radians, 0.7), TOLERANCE);
    }

    @Test
    void armGravityAgreesWithTheIdentifiedRotaryModel() {
        double offset = 0.3;
        FeedforwardGains arm = FeedforwardGains.arm(0.0, 0.0, KG, Units.Radians.of(offset)).withGravityAngle(Units.Rotations);
        for (double rotations : new double[] {-0.4, 0.0, 0.125, 0.5}) {
            assertEquals(Identification.gravityVolts(MotionKind.ROTARY, KG, offset, rotations),
                    gravity(arm, rotations), TOLERANCE);
        }
    }

    @Test
    void typedDegreesBindingUsesItsUnit() {
        ControlBinding<?> control = Controls.position(motor(), Units.Degrees)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 2).measures(Units.Degrees))
                .armFf(0.0, 0.0, KG);
        FeedforwardGains gains = feedforward(control);
        assertTrue(gains.isArm());
        assertSame(Units.Degrees, gains.gravityAngle());
        assertEquals(KG * Math.cos(Math.toRadians(60.0)), gravity(gains, 60.0), TOLERANCE);
    }

    @Test
    void typedEncoderDeclaredInDegreesRotationsAndRadiansIsUsed() {
        assertEquals(Units.Degrees, boundAngle(Units.Degrees));
        assertEquals(Units.Rotations, boundAngle(Units.Rotations));
        assertEquals(Units.Radians, boundAngle(Units.Radians));
    }

    @Test
    void encoderMeasuringDegreesIsUsedForAnUntypedBinding() {
        ControlBinding<?> control = Controls.position(motor(), Units.Degrees)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 4).measures(Units.Degrees))
                .armFf(0.0, 0.0, KG);
        assertSame(Units.Degrees, feedforward(control).gravityAngle());
    }

    @Test
    void armFfRejectsVelocityControls() {
        ControlBinding<?> velocity = Controls.velocity(motor(), Units.Rotations)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 5).measures(Units.Rotations));
        assertThrows(IllegalArgumentException.class, () -> velocity.armFf(0.0, 0.0, KG));
    }

    @Test
    void armFfRejectsLinearUnits() {
        ControlBinding<?> linear = Controls.position(motor(), Units.Meters)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 6).measures(Units.Meters));
        assertThrows(IllegalArgumentException.class, () -> linear.armFf(0.0, 0.0, KG));
    }

    @Test
    void armFfRejectsEncodersThatMeasureADistance() {
        ControlBinding<?> linear = Controls.position(motor(), Units.Meters)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 7).measures(Units.Meters));
        IllegalArgumentException raised = assertThrows(IllegalArgumentException.class,
                () -> linear.armFf(0.0, 0.0, KG));
        assertTrue(raised.getMessage().contains("needs an angle"));
    }

    @Test
    void armFfUsesRotationsForAnEncoderThatDeclaresNoAngleUnit() {
        ControlBinding<?> rotations = Controls.position(motor(), Units.Rotations)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 7))
                .armFf(0.0, 0.0, KG);
        assertEquals(Units.Rotations, feedforward(rotations).gravityAngle());
    }

    @Test
    void armFfRejectsWithoutFeedback() {
        assertThrows(IllegalArgumentException.class, () -> Controls.position(motor(), Units.Degrees).armFf(0, 0, KG));
    }

    @Test
    void plainFfKeepsConstantGravity() {
        ControlBinding<?> control = Controls.position(motor(), Units.Degrees)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 8).measures(Units.Degrees))
                .ff(0.0, 0.0, KG);
        FeedforwardGains gains = feedforward(control);
        assertFalse(gains.isArm());
        assertEquals(ControlLoopRole.DEVICE_CONFIGURABLE, gains.role());
        assertEquals(KG, gravity(gains, 0.0), TOLERANCE);
        assertEquals(KG, gravity(gains, 90.0), TOLERANCE);
    }

    @Test
    void armFormReplacesPlainFeedforward() {
        ControlBinding<?> control = Controls.position(motor(), Units.Degrees)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 9).measures(Units.Degrees))
                .ff(0.0, 0.0, KG)
                .armFf(0.0, 0.0, KG);
        assertEquals(1, control.loops().stream().filter(loop -> loop instanceof FeedforwardGains).count());
        assertTrue(feedforward(control).isArm());
        assertEquals(ControlLoopRole.ARBITRARY_FEEDFORWARD, feedforward(control).role());
    }

    @Test
    void armGainsKeepTheirFormWhenAccelerationIsSet() {
        FeedforwardGains arm = FeedforwardGains.arm(0.0, 0.0, KG, Units.Radians.of(0.1)).withGravityAngle(Units.Degrees);
        FeedforwardGains withAcceleration = arm.acceleration(0.5);
        assertTrue(withAcceleration.isArm());
        assertEquals(0.1, withAcceleration.gravityOffset().in(Units.Radians), TOLERANCE);
        assertSame(Units.Degrees, withAcceleration.gravityAngle());
    }

    @Test
    void unboundArmGainsAreRejectedAtRuntime() {
        FeedforwardGains unbound = FeedforwardGains.arm(0.0, 0.0, KG);
        assertThrows(IllegalStateException.class, () -> gravity(unbound, 0.0));
        assertThrows(IllegalStateException.class, () -> FeedforwardGains.of(0.0, 0.0, KG).withGravityAngle(Units.Degrees));
    }

    /** Angle unit the arm form binds to for a typed binding whose encoder declares the same {@code measures(...)}. */
    private static AngleUnit boundAngle(AngleUnit units) {
        ControlBinding<?> control = Controls.position(motor(), units)
                .feedback(EncoderDevice.of(EncoderKinds.CANCODER, 10).measures(units))
                .armFf(0.0, 0.0, KG);
        return feedforward(control).gravityAngle();
    }

    private static FeedforwardGains feedforward(ControlBinding<?> control) {
        List<ControlLoop> loops = control.loops();
        return loops.stream()
                .filter(FeedforwardGains.class::isInstance)
                .map(FeedforwardGains.class::cast)
                .findFirst()
                .orElseThrow();
    }
}
