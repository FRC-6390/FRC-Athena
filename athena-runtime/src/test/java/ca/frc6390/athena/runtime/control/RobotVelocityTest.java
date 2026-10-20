package ca.frc6390.athena.runtime.control;

import edu.wpi.first.math.geometry.Rotation2d;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class RobotVelocityTest {
    @Test
    void immutableMathPreservesFrameAndComponents() {
        RobotVelocity first = RobotVelocity.robot(MetersPerSecond.of(2.0), MetersPerSecond.of(-1.0), RadiansPerSecond.of(0.5));
        RobotVelocity second = RobotVelocity.robot(MetersPerSecond.of(-0.5), MetersPerSecond.of(3.0), RadiansPerSecond.of(-0.25));

        assertEquals(RobotVelocity.robot(MetersPerSecond.of(1.5), MetersPerSecond.of(2.0), RadiansPerSecond.of(0.25)), first.plus(second));
        assertEquals(RobotVelocity.robot(MetersPerSecond.of(2.5), MetersPerSecond.of(-4.0), RadiansPerSecond.of(0.75)), first.minus(second));
        assertEquals(RobotVelocity.robot(MetersPerSecond.of(4.0), MetersPerSecond.of(-2.0), RadiansPerSecond.of(1.0)), first.times(2.0));
        assertEquals(first, first.times(2.0).div(2.0));
        assertEquals(RobotVelocity.robot(MetersPerSecond.of(-2.0), MetersPerSecond.of(1.0), RadiansPerSecond.of(-0.5)), first.unaryMinus());
        assertEquals(RobotVelocity.robot(MetersPerSecond.of(-2.0), MetersPerSecond.of(1.0), RadiansPerSecond.of(-0.5)), first.inverted());
        assertEquals(RobotVelocity.robot(MetersPerSecond.of(-2.0), MetersPerSecond.of(-1.0), RadiansPerSecond.of(-0.5)), first.inverted(true, false, true));
        assertEquals(RobotVelocity.robot(MetersPerSecond.of(2.0), MetersPerSecond.of(-1.0), RadiansPerSecond.of(0.0)), first.linearOnly());
        assertEquals(RobotVelocity.robot(MetersPerSecond.of(0.0), MetersPerSecond.of(0.0), RadiansPerSecond.of(0.5)), first.angularOnly());
        assertEquals(Math.sqrt(5.0), first.linearMagnitude().in(MetersPerSecond), 1.0e-9);
    }

    @Test
    void conversionRoundTripsAndMismatchedDirectMathFails() {
        RobotVelocity field = RobotVelocity.field(MetersPerSecond.of(2.0), MetersPerSecond.of(-0.5), RadiansPerSecond.of(0.25));
        RobotVelocity robot = field.fieldToRobot(Rotation2d.fromRadians(1.2));

        assertEquals(field.raw().xMetersPerSecond(), robot.robotToField(Rotation2d.fromRadians(1.2)).raw().xMetersPerSecond(), 1.0e-9);
        assertEquals(field.raw().yMetersPerSecond(), robot.robotToField(Rotation2d.fromRadians(1.2)).raw().yMetersPerSecond(), 1.0e-9);
        assertEquals(field.raw().angularRadiansPerSecond(), robot.raw().angularRadiansPerSecond(), 1.0e-9);
        assertThrows(IllegalArgumentException.class, () -> field.plus(robot));
        assertThrows(IllegalStateException.class, () -> field.robotToField(Rotation2d.fromRadians(0.0)));
    }

    @Test
    void typedFactoriesMatchRawPrimitiveConstruction() {
        // Typed WPILib factories must store the same primitives as the raw hot-path builder.
        // Degrees per second must become radians per second (90 deg/s = pi/2 rad/s).
        RobotVelocity typed = RobotVelocity.field(
                MetersPerSecond.of(1.2), MetersPerSecond.of(-0.5), DegreesPerSecond.of(90.0));
        RobotVelocity raw = RobotVelocity.Raw.of(1.2, -0.5, Math.toRadians(90.0), VelocityFrame.FIELD);

        assertEquals(VelocityFrame.FIELD, typed.frame());
        assertEquals(raw.raw().xMetersPerSecond(), typed.raw().xMetersPerSecond(), 1.0e-12);
        assertEquals(raw.raw().yMetersPerSecond(), typed.raw().yMetersPerSecond(), 1.0e-12);
        assertEquals(raw.raw().angularRadiansPerSecond(), typed.raw().angularRadiansPerSecond(), 1.0e-12);
        assertEquals(Math.PI / 2.0, typed.angular().in(RadiansPerSecond), 1.0e-12);
        assertEquals(1.2, typed.x().in(MetersPerSecond), 1.0e-12);
    }

    @Test
    void typedHeadingConversionMatchesPrimitiveRotation() {
        // Rotation2d.fromDegrees(30) must rotate exactly like the primitive field-to-robot formula at 30 degrees.
        RobotVelocity field = RobotVelocity.field(MetersPerSecond.of(2.0), MetersPerSecond.of(-0.5), RadiansPerSecond.of(0.25));
        double heading = Math.toRadians(30.0);
        double expectedX = 2.0 * Math.cos(heading) + -0.5 * Math.sin(heading);
        double expectedY = -2.0 * Math.sin(heading) + -0.5 * Math.cos(heading);

        RobotVelocity robot = field.fieldToRobot(Rotation2d.fromDegrees(30.0));

        assertEquals(VelocityFrame.ROBOT, robot.frame());
        assertEquals(expectedX, robot.raw().xMetersPerSecond(), 1.0e-12);
        assertEquals(expectedY, robot.raw().yMetersPerSecond(), 1.0e-12);
        assertEquals(0.25, robot.angular().in(RadiansPerSecond), 1.0e-12);
    }

    @Test
    void poolCombinesFramesWeightsAndDynamicEnablement() {
        boolean[] assistEnabled = {true};
        double[] driverX = {2.0};
        RobotVelocityPool pool = new RobotVelocityPool();
        RobotVelocityPool.Channel driver = pool.channel()
                .set(() -> RobotVelocity.field(MetersPerSecond.of(driverX[0]), MetersPerSecond.of(0.0), RadiansPerSecond.of(0.0)));
        RobotVelocityPool.Channel assist = pool.channel()
                .set(RobotVelocity.angular(RadiansPerSecond.of(1.0)))
                .weight(0.5)
                .enabled(() -> assistEnabled[0]);

        RobotVelocity combined = pool.robotRelative(Rotation2d.fromRadians(Math.PI / 2.0));
        assertEquals(0.0, combined.raw().xMetersPerSecond(), 1.0e-9);
        assertEquals(-2.0, combined.raw().yMetersPerSecond(), 1.0e-9);
        assertEquals(0.5, combined.raw().angularRadiansPerSecond(), 1.0e-9);
        assertTrue(driver.isActive());
        assertTrue(assist.isActive());

        driverX[0] = 3.0;
        assistEnabled[0] = false;
        combined = pool.robotRelative(Rotation2d.fromRadians(Math.PI / 2.0));
        assertEquals(-3.0, combined.raw().yMetersPerSecond(), 1.0e-9);
        assertEquals(0.0, combined.raw().angularRadiansPerSecond(), 1.0e-9);
        assertFalse(assist.isActive());

        driver.clear();
        assertEquals(RobotVelocity.zero(), pool.robotRelative(Rotation2d.fromRadians(0.0)));
    }

    @Test
    void interpolationAndClampingBehaveLikeGeometryValues() {
        RobotVelocity start = RobotVelocity.field(MetersPerSecond.of(0.0), MetersPerSecond.of(2.0), RadiansPerSecond.of(-2.0));
        RobotVelocity end = RobotVelocity.field(MetersPerSecond.of(4.0), MetersPerSecond.of(0.0), RadiansPerSecond.of(2.0));

        assertEquals(RobotVelocity.field(MetersPerSecond.of(1.0), MetersPerSecond.of(1.5), RadiansPerSecond.of(-1.0)), start.interpolate(end, 0.25));
        assertEquals(start, start.interpolate(end, -1.0));
        assertEquals(end, start.interpolate(end, 2.0));
        assertEquals(RobotVelocity.field(MetersPerSecond.of(0.0), MetersPerSecond.of(1.0), RadiansPerSecond.of(-0.5)), start.clamp(MetersPerSecond.of(1.0), RadiansPerSecond.of(0.5)));
    }

    @Test
    void angularChannelReplacesPooledRotationAndReleasesCleanly() {
        RobotVelocityPool pool = new RobotVelocityPool();
        pool.channel().set(RobotVelocity.robot(MetersPerSecond.of(2.0), MetersPerSecond.of(-1.0), RadiansPerSecond.of(0.75)));
        RobotVelocityPool.AngularChannel aim = pool.angularChannel();

        aim.apply(-1.25);
        assertEquals(RobotVelocity.robot(MetersPerSecond.of(2.0), MetersPerSecond.of(-1.0), RadiansPerSecond.of(-1.25)), pool.robotRelative(Rotation2d.fromRadians(0.0)));
        assertTrue(aim.isActive());

        aim.release();
        assertEquals(RobotVelocity.robot(MetersPerSecond.of(2.0), MetersPerSecond.of(-1.0), RadiansPerSecond.of(0.75)), pool.robotRelative(Rotation2d.fromRadians(0.0)));
        assertFalse(aim.isActive());
    }

    @Test
    void mostRecentlyAppliedAngularChannelOwnsRotation() {
        RobotVelocityPool pool = new RobotVelocityPool();
        RobotVelocityPool.AngularChannel first = pool.angularChannel();
        RobotVelocityPool.AngularChannel second = pool.angularChannel();

        first.apply(1.0);
        second.apply(2.0);
        assertEquals(2.0, pool.robotRelative(Rotation2d.fromRadians(0.0)).raw().angularRadiansPerSecond(), 1.0e-9);

        first.apply(3.0);
        assertEquals(3.0, pool.robotRelative(Rotation2d.fromRadians(0.0)).raw().angularRadiansPerSecond(), 1.0e-9);
    }

    @Test
    void channelInversionCanReverseSelectedVelocityComponents() {
        RobotVelocityPool pool = new RobotVelocityPool();
        pool.channel()
                .set(RobotVelocity.robot(MetersPerSecond.of(2.0), MetersPerSecond.of(-3.0), RadiansPerSecond.of(0.75)))
                .inverted(true, false, true);

        assertEquals(RobotVelocity.robot(MetersPerSecond.of(-2.0), MetersPerSecond.of(-3.0), RadiansPerSecond.of(-0.75)), pool.robotRelative(Rotation2d.fromRadians(0.0)));
    }

    @Test
    void angularControlChannelInvertsAppliedPidEffort() {
        RobotVelocityPool pool = new RobotVelocityPool();
        RobotVelocityPool.AngularChannel aim = pool.angularChannel().inverted();

        aim.apply(1.25);
        assertTrue(aim.isInverted());
        assertEquals(-1.25, aim.angularVelocity().in(RadiansPerSecond), 1.0e-9);
        assertEquals(-1.25, pool.robotRelative(Rotation2d.fromRadians(0.0)).raw().angularRadiansPerSecond(), 1.0e-9);

        aim.inverted(false);
        assertEquals(1.25, pool.robotRelative(Rotation2d.fromRadians(0.0)).raw().angularRadiansPerSecond(), 1.0e-9);
    }
}
