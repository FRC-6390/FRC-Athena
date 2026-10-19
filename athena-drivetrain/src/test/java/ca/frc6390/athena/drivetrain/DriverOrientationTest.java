package ca.frc6390.athena.drivetrain;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.drivetrain.DriverOrientation.Alliance;
import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.control.RobotVelocityPool;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.junit.jupiter.api.Test;

class DriverOrientationTest {
    private static final double PI = Math.PI;
    private static final double EPS = 1.0e-9;

    @Test
    void blueForwardIsFieldPlusXAndDriverHeadingIsTheFieldHeading() {
        Rig rig = new Rig(Optional.of(Alliance.BLUE), false);
        rig.pose = new PoseSnapshot(2.0, 3.0, 0.4);

        assertEquals(0.0, rig.orientation.forwardRadians(), EPS);
        assertAngle(0.4, rig.orientation.driverHeadingRadians());
        assertTrue(rig.orientation.allianceKnown());
    }

    @Test
    void redForwardIsOppositeAndRobotFacingBlueIsBackward() {
        Rig rig = new Rig(Optional.of(Alliance.RED), false);
        rig.pose = new PoseSnapshot(2.0, 3.0, 0.0);

        assertAngle(PI, rig.orientation.forwardRadians());
        // Facing field +X on red means facing the blue wall, away from the driver.
        assertAngle(PI, rig.orientation.driverHeadingRadians());
    }

    @Test
    void redFieldFacingDriverForwardGivesZeroDriverHeading() {
        Rig rig = new Rig(Optional.of(Alliance.RED), true);
        rig.pose = new PoseSnapshot(2.0, 3.0, PI);

        assertAngle(0.0, rig.orientation.driverHeadingRadians());
    }

    @Test
    void autoToTeleopNeedsNoRealignmentOnEitherAlliance() {
        for (Alliance alliance : new Alliance[] {Alliance.BLUE, Alliance.RED}) {
            Rig rig = new Rig(Optional.of(alliance), true);
            // Auto ends with the localized pose; teleop reads the same field heading.
            rig.pose = new PoseSnapshot(10.0, 4.0, 1.0);
            double forward = rig.orientation.forwardRadians();

            assertAngle(1.0 - forward, rig.orientation.driverHeadingRadians());
        }
    }

    @Test
    void stickForwardDrivesAwayFromOwnDriverStationOnBothAlliances() {
        // Blue, robot facing field +Y: forward stick goes to field +X.
        assertFieldDirection(Alliance.BLUE, PI / 2.0, 1.0, 0.0, 1.0, 0.0);
        // Red, robot facing field +X (toward blue): forward stick goes to field -X.
        assertFieldDirection(Alliance.RED, 0.0, 1.0, 0.0, -1.0, 0.0);
    }

    @Test
    void unknownAllianceAtBootAssumesBlueUntilTheDriverStationReportsIt() {
        Rig rig = new Rig(Optional.empty(), false);
        rig.pose = new PoseSnapshot(0.0, 0.0, 0.0);

        assertFalse(rig.orientation.allianceKnown());
        assertEquals(0.0, rig.orientation.forwardRadians(), EPS);

        rig.alliance = Optional.of(Alliance.RED);

        assertTrue(rig.orientation.allianceKnown());
        assertAngle(PI, rig.orientation.forwardRadians());
    }

    @Test
    void driverResetMakesCurrentFacingForwardWithoutTouchingThePose() {
        Rig rig = new Rig(Optional.of(Alliance.RED), false);
        rig.pose = new PoseSnapshot(2.0, 3.0, 0.7);

        rig.orientation.captureForwardFromCurrentHeading();

        assertAngle(0.7, rig.orientation.forwardRadians());
        assertAngle(0.0, rig.orientation.driverHeadingRadians());
        assertEquals(0, rig.resets.size(), "reset must not change the field pose");

        // The robot turns later in teleop; the offset stays relative to the reset facing.
        rig.pose = new PoseSnapshot(2.0, 3.0, 1.2);
        assertAngle(0.5, rig.orientation.driverHeadingRadians());
    }

    @Test
    void restoringAllianceForwardClearsTheDriverReset() {
        Rig rig = new Rig(Optional.of(Alliance.BLUE), false);
        rig.pose = new PoseSnapshot(0.0, 0.0, 0.7);
        rig.orientation.captureForwardFromCurrentHeading();

        rig.orientation.clearDriverOffset();

        assertAngle(0.0, rig.orientation.forwardRadians());
        assertAngle(0.7, rig.orientation.driverHeadingRadians());
    }

    @Test
    void midAutoPoseResetChangesTheFieldHeadingButNotTheDriverForward() {
        Rig rig = new Rig(Optional.of(Alliance.RED), true);
        rig.pose = new PoseSnapshot(5.0, 5.0, 0.3);
        double forward = rig.orientation.forwardRadians();

        // A path resets odometry to its correct field pose mid-auto.
        rig.pose = new PoseSnapshot(8.0, 2.0, -2.0);

        assertAngle(forward, rig.orientation.forwardRadians());
        assertAngle(-2.0 - forward, rig.orientation.driverHeadingRadians());
    }

    @Test
    void unseededBootAssumesDriverForwardWhileKeepingTranslation() {
        Rig rig = new Rig(Optional.of(Alliance.RED), false);
        rig.pose = new PoseSnapshot(3.0, 4.0, 0.2);

        rig.orientation.assumeDriverForwardBinding().apply(null);

        assertEquals(1, rig.resets.size());
        PoseSnapshot reset = rig.resets.get(0);
        assertEquals(3.0, reset.xMeters(), EPS);
        assertEquals(4.0, reset.yMeters(), EPS);
        assertAngle(PI, reset.headingRadians());
    }

    @Test
    void seededPoseIsNotOverwrittenAtTeleopStart() {
        Rig rig = new Rig(Optional.of(Alliance.BLUE), true);
        rig.pose = new PoseSnapshot(3.0, 4.0, 0.2);

        rig.orientation.assumeDriverForwardBinding().apply(null);

        assertTrue(rig.resets.isEmpty());
    }

    private static void assertFieldDirection(
            Alliance alliance, double poseHeading, double stickX, double stickY, double expectedX, double expectedY) {
        Rig rig = new Rig(Optional.of(alliance), true);
        rig.pose = new PoseSnapshot(0.0, 0.0, poseHeading);
        RobotVelocityPool pool = new RobotVelocityPool();
        pool.channel().set(RobotVelocity.field(stickX, stickY, 0.0));

        RobotVelocity robot = pool.robotRelative(rig.orientation.driverHeadingRadians());
        double cos = Math.cos(poseHeading);
        double sin = Math.sin(poseHeading);
        double fieldX = robot.xMetersPerSecond() * cos - robot.yMetersPerSecond() * sin;
        double fieldY = robot.xMetersPerSecond() * sin + robot.yMetersPerSecond() * cos;

        assertEquals(expectedX, fieldX, EPS);
        assertEquals(expectedY, fieldY, EPS);
    }

    private static void assertAngle(double expected, double actual) {
        double delta = Math.atan2(Math.sin(actual - expected), Math.cos(actual - expected));
        assertEquals(0.0, delta, EPS, "expected " + expected + " but was " + actual);
    }

    private static final class Rig {
        final List<PoseSnapshot> resets = new ArrayList<>();
        Optional<Alliance> alliance;
        PoseSnapshot pose = new PoseSnapshot(0.0, 0.0, 0.0);
        final DriverOrientation orientation;

        Rig(Optional<Alliance> alliance, boolean seeded) {
            this.alliance = alliance;
            this.orientation = DriverOrientation.of(
                    () -> pose,
                    () -> this.alliance,
                    () -> seeded,
                    next -> {
                        resets.add(next);
                        return ActionBinding.run(() -> pose = next);
                    });
        }
    }
}
