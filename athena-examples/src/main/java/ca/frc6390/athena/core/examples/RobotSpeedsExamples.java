package ca.frc6390.athena.core.examples;

import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.core.RobotSpeeds.BlendMode;
import ca.frc6390.athena.core.RobotSpeeds.SpeedAxis;

/**
 * Examples showing common RobotSpeeds blend patterns.
 */
public final class RobotSpeedsExamples {
    private RobotSpeedsExamples() {}

    public static RobotSpeeds createDefaultProfile(double maxVelocity, double maxAngularVelocity) {
        return new RobotSpeeds(maxVelocity, maxAngularVelocity);
    }

    /**
     * Replaces default output blending with:
     * 1) drive translation averaged with auto translation
     * 2) drive rotational command plus feedback rotational correction
     */
    public static RobotSpeeds configureDriveAutoAverageForTranslation(RobotSpeeds speeds) {
        speeds.clearBlends();
        speeds.blend(RobotSpeeds.DRIVE_SOURCE, RobotSpeeds.DRIVE_SOURCE, RobotSpeeds.AUTO_SOURCE, BlendMode.AVERAGE,
                SpeedAxis.X, SpeedAxis.Y);
        speeds.blendToOutput(RobotSpeeds.DRIVE_SOURCE, BlendMode.ADD, SpeedAxis.ALL);
        speeds.blendToOutput(RobotSpeeds.FEEDBACK_SOURCE, BlendMode.ADD, SpeedAxis.Theta);
        return speeds;
    }

    /**
     * Adds a heading-assist source that can override the previously blended theta output.
     */
    public static RobotSpeeds configureHeadingAssistOverride(RobotSpeeds speeds, String assistSource) {
        speeds.registerSpeedSource(assistSource, true);
        speeds.blendToOutput(assistSource, BlendMode.B_SUPERSEDES_A, SpeedAxis.Theta);
        return speeds;
    }
}
