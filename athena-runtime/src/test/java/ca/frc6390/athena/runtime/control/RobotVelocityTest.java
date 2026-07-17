package ca.frc6390.athena.runtime.control;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class RobotVelocityTest {
    @Test
    void immutableMathPreservesFrameAndComponents() {
        RobotVelocity first = RobotVelocity.robot(2.0, -1.0, 0.5);
        RobotVelocity second = RobotVelocity.robot(-0.5, 3.0, -0.25);

        assertEquals(RobotVelocity.robot(1.5, 2.0, 0.25), first.plus(second));
        assertEquals(RobotVelocity.robot(2.5, -4.0, 0.75), first.minus(second));
        assertEquals(RobotVelocity.robot(4.0, -2.0, 1.0), first.times(2.0));
        assertEquals(first, first.times(2.0).div(2.0));
        assertEquals(RobotVelocity.robot(-2.0, 1.0, -0.5), first.unaryMinus());
        assertEquals(RobotVelocity.robot(2.0, -1.0, 0.0), first.linearOnly());
        assertEquals(RobotVelocity.robot(0.0, 0.0, 0.5), first.angularOnly());
        assertEquals(Math.sqrt(5.0), first.linearMagnitude(), 1.0e-9);
    }

    @Test
    void conversionRoundTripsAndMismatchedDirectMathFails() {
        RobotVelocity field = RobotVelocity.field(2.0, -0.5, 0.25);
        RobotVelocity robot = field.fieldToRobot(1.2);

        assertEquals(field.xMetersPerSecond(), robot.robotToField(1.2).xMetersPerSecond(), 1.0e-9);
        assertEquals(field.yMetersPerSecond(), robot.robotToField(1.2).yMetersPerSecond(), 1.0e-9);
        assertEquals(field.angularRadiansPerSecond(), robot.angularRadiansPerSecond(), 1.0e-9);
        assertThrows(IllegalArgumentException.class, () -> field.plus(robot));
        assertThrows(IllegalStateException.class, () -> field.robotToField(0.0));
    }

    @Test
    void poolCombinesFramesWeightsAndDynamicEnablement() {
        boolean[] assistEnabled = {true};
        double[] driverX = {2.0};
        RobotVelocityPool pool = new RobotVelocityPool();
        RobotVelocityPool.Channel driver = pool.channel()
                .set(() -> RobotVelocity.field(driverX[0], 0.0, 0.0));
        RobotVelocityPool.Channel assist = pool.channel()
                .set(RobotVelocity.angular(1.0))
                .weight(0.5)
                .enabled(() -> assistEnabled[0]);

        RobotVelocity combined = pool.robotRelative(Math.PI / 2.0);
        assertEquals(0.0, combined.xMetersPerSecond(), 1.0e-9);
        assertEquals(-2.0, combined.yMetersPerSecond(), 1.0e-9);
        assertEquals(0.5, combined.angularRadiansPerSecond(), 1.0e-9);
        assertTrue(driver.isActive());
        assertTrue(assist.isActive());

        driverX[0] = 3.0;
        assistEnabled[0] = false;
        combined = pool.robotRelative(Math.PI / 2.0);
        assertEquals(-3.0, combined.yMetersPerSecond(), 1.0e-9);
        assertEquals(0.0, combined.angularRadiansPerSecond(), 1.0e-9);
        assertFalse(assist.isActive());

        driver.clear();
        assertEquals(RobotVelocity.zero(), pool.robotRelative(0.0));
    }

    @Test
    void interpolationAndClampingBehaveLikeGeometryValues() {
        RobotVelocity start = RobotVelocity.field(0.0, 2.0, -2.0);
        RobotVelocity end = RobotVelocity.field(4.0, 0.0, 2.0);

        assertEquals(RobotVelocity.field(1.0, 1.5, -1.0), start.interpolate(end, 0.25));
        assertEquals(start, start.interpolate(end, -1.0));
        assertEquals(end, start.interpolate(end, 2.0));
        assertEquals(RobotVelocity.field(0.0, 1.0, -0.5), start.clamp(1.0, 0.5));
    }

    @Test
    void angularChannelReplacesPooledRotationAndReleasesCleanly() {
        RobotVelocityPool pool = new RobotVelocityPool();
        pool.channel().set(RobotVelocity.robot(2.0, -1.0, 0.75));
        RobotVelocityPool.AngularChannel aim = pool.angularChannel();

        aim.apply(-1.25);
        assertEquals(RobotVelocity.robot(2.0, -1.0, -1.25), pool.robotRelative(0.0));
        assertTrue(aim.isActive());

        aim.release();
        assertEquals(RobotVelocity.robot(2.0, -1.0, 0.75), pool.robotRelative(0.0));
        assertFalse(aim.isActive());
    }

    @Test
    void mostRecentlyAppliedAngularChannelOwnsRotation() {
        RobotVelocityPool pool = new RobotVelocityPool();
        RobotVelocityPool.AngularChannel first = pool.angularChannel();
        RobotVelocityPool.AngularChannel second = pool.angularChannel();

        first.apply(1.0);
        second.apply(2.0);
        assertEquals(2.0, pool.robotRelative(0.0).angularRadiansPerSecond(), 1.0e-9);

        first.apply(3.0);
        assertEquals(3.0, pool.robotRelative(0.0).angularRadiansPerSecond(), 1.0e-9);
    }
}
