package ca.frc6390.athena.pathplanner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import org.junit.jupiter.api.Test;

final class InvertedFeedbackPPHolonomicDriveControllerTest {

    @Test
    void keepsFeedforwardWhenAllAxesAreAtSetpointWithInversionEnabled() {
        InvertedFeedbackPPHolonomicDriveController controller = new InvertedFeedbackPPHolonomicDriveController(
                new PIDConstants(1.0, 0.0, 0.0),
                new PIDConstants(1.0, 0.0, 0.0),
                true,
                true,
                true);

        Pose2d currentPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(35.0));
        PathPlannerTrajectoryState target = state(
                0.0,
                0.0,
                35.0,
                1.2,
                -0.4,
                0.6);

        ChassisSpeeds robotRelative = controller.calculateRobotRelativeSpeeds(currentPose, target);
        ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, currentPose.getRotation());

        assertEquals(1.2, fieldRelative.vxMetersPerSecond, 1e-9);
        assertEquals(-0.4, fieldRelative.vyMetersPerSecond, 1e-9);
        assertEquals(0.6, fieldRelative.omegaRadiansPerSecond, 1e-9);
    }

    @Test
    void invertsOnlyRotationFeedbackWhenConfigured() {
        PIDConstants translationPid = new PIDConstants(0.0, 0.0, 0.0);
        PIDConstants rotationPid = new PIDConstants(1.0, 0.0, 0.0);

        PPHolonomicDriveController base = new PPHolonomicDriveController(translationPid, rotationPid);
        InvertedFeedbackPPHolonomicDriveController inverted = new InvertedFeedbackPPHolonomicDriveController(
                translationPid,
                rotationPid,
                false,
                false,
                true);

        Pose2d currentPose = new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0.0));
        PathPlannerTrajectoryState target = state(
                0.0,
                0.0,
                20.0,
                0.0,
                0.0,
                0.0);

        ChassisSpeeds baseOut = base.calculateRobotRelativeSpeeds(currentPose, target);
        ChassisSpeeds invertedOut = inverted.calculateRobotRelativeSpeeds(currentPose, target);

        assertTrue(baseOut.omegaRadiansPerSecond > 0.0);
        assertTrue(invertedOut.omegaRadiansPerSecond < 0.0);
        assertEquals(baseOut.vxMetersPerSecond, invertedOut.vxMetersPerSecond, 1e-9);
        assertEquals(baseOut.vyMetersPerSecond, invertedOut.vyMetersPerSecond, 1e-9);
        assertEquals(-baseOut.omegaRadiansPerSecond, invertedOut.omegaRadiansPerSecond, 1e-9);
    }

    private static PathPlannerTrajectoryState state(
            double xMeters,
            double yMeters,
            double headingDegrees,
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond) {
        PathPlannerTrajectoryState state = new PathPlannerTrajectoryState();
        state.pose = new Pose2d(xMeters, yMeters, Rotation2d.fromDegrees(headingDegrees));
        state.fieldSpeeds = new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
        return state;
    }
}
