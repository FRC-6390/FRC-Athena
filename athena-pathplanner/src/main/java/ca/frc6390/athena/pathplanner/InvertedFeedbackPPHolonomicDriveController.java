package ca.frc6390.athena.pathplanner;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/**
 * PathPlanner holonomic controller variant that can invert selected feedback axes while preserving
 * trajectory-provided feedforward terms.
 */
final class InvertedFeedbackPPHolonomicDriveController extends PPHolonomicDriveController {
    private final boolean invertXFeedback;
    private final boolean invertYFeedback;
    private final boolean invertOmegaFeedback;

    InvertedFeedbackPPHolonomicDriveController(
            PIDConstants translationConstants,
            PIDConstants rotationConstants,
            boolean invertXFeedback,
            boolean invertYFeedback,
            boolean invertOmegaFeedback) {
        super(translationConstants, rotationConstants);
        this.invertXFeedback = invertXFeedback;
        this.invertYFeedback = invertYFeedback;
        this.invertOmegaFeedback = invertOmegaFeedback;
    }

    InvertedFeedbackPPHolonomicDriveController(
            PIDConstants translationConstants,
            PIDConstants rotationConstants,
            double periodSeconds,
            boolean invertXFeedback,
            boolean invertYFeedback,
            boolean invertOmegaFeedback) {
        super(translationConstants, rotationConstants, periodSeconds);
        this.invertXFeedback = invertXFeedback;
        this.invertYFeedback = invertYFeedback;
        this.invertOmegaFeedback = invertOmegaFeedback;
    }

    @Override
    public ChassisSpeeds calculateRobotRelativeSpeeds(Pose2d currentPose, PathPlannerTrajectoryState targetState) {
        ChassisSpeeds base = super.calculateRobotRelativeSpeeds(currentPose, targetState);
        ChassisSpeeds fieldRelativeBase = ChassisSpeeds.fromRobotRelativeSpeeds(base, currentPose.getRotation());
        ChassisSpeeds feedforward = targetState.fieldSpeeds;

        double adjustedVx = applyFeedbackSign(fieldRelativeBase.vxMetersPerSecond, feedforward.vxMetersPerSecond, invertXFeedback);
        double adjustedVy = applyFeedbackSign(fieldRelativeBase.vyMetersPerSecond, feedforward.vyMetersPerSecond, invertYFeedback);
        double adjustedOmega = applyFeedbackSign(
                fieldRelativeBase.omegaRadiansPerSecond,
                feedforward.omegaRadiansPerSecond,
                invertOmegaFeedback);

        return ChassisSpeeds.fromFieldRelativeSpeeds(
                adjustedVx,
                adjustedVy,
                adjustedOmega,
                currentPose.getRotation());
    }

    private static double applyFeedbackSign(double combined, double feedforward, boolean invertFeedback) {
        if (!invertFeedback) {
            return combined;
        }
        return (2.0 * feedforward) - combined;
    }
}
