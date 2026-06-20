package ca.frc6390.athena.examples;

import ca.frc6390.athena.runtime.control.RobotSpeeds;
import ca.frc6390.athena.runtime.control.RobotVelocity;

/**
 * Robot speed source blending examples.
 */
public final class RobotSpeedsExample {
    private RobotSpeedsExample() {
    }

    /**
     * Creates the default drive, autonomous, and feedback speed profile.
     *
     * @return speed blender
     */
    public static RobotSpeeds defaultProfile() {
        return new RobotSpeeds(3.0, 2.0);
    }

    /**
     * Configures a profile that averages drive and autonomous translation.
     *
     * @return configured speed blender
     */
    public static RobotSpeeds driveAutoAverageForTranslation() {
        return new RobotSpeeds(10.0, 10.0)
                .clearBlends()
                .blend(RobotSpeeds.DRIVE_SOURCE, RobotSpeeds.DRIVE_SOURCE, RobotSpeeds.AUTO_SOURCE,
                        RobotSpeeds.BlendMode.AVERAGE, RobotSpeeds.SpeedAxis.X, RobotSpeeds.SpeedAxis.Y)
                .blendToOutput(RobotSpeeds.DRIVE_SOURCE, RobotSpeeds.BlendMode.ADD, RobotSpeeds.SpeedAxis.ALL)
                .blendToOutput(RobotSpeeds.FEEDBACK_SOURCE, RobotSpeeds.BlendMode.ADD, RobotSpeeds.SpeedAxis.THETA);
    }

    /**
     * Configures heading assist to supersede theta output.
     *
     * @return configured speed blender
     */
    public static RobotSpeeds headingAssistOverride() {
        return new RobotSpeeds(10.0, 10.0)
                .registerSource("assist")
                .blendToOutput("assist", RobotSpeeds.BlendMode.B_SUPERSEDES_A, RobotSpeeds.SpeedAxis.THETA);
    }

    /**
     * Converts a field-relative drive command with a 90 degree robot heading.
     *
     * @return robot-relative velocity
     */
    public static RobotVelocity fieldRelativeExample() {
        return new RobotSpeeds(10.0, 10.0)
                .setFieldRelativeSpeeds(RobotSpeeds.DRIVE_SOURCE, 2.0, 0.0, 1.0)
                .calculate(Math.PI / 2.0);
    }
}
