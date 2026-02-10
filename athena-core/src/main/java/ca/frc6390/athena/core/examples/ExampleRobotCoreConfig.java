package ca.frc6390.athena.core.examples;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrainConfig;
import ca.frc6390.athena.hardware.imu.AthenaImu;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;

/**
 * Minimal compile-time example showing the intended "Superstructure-style" look and feel.
 *
 * <p>Teams are expected to fill in module configs, pose configs, and mechanism configs for their robot.</p>
 */
public final class ExampleRobotCoreConfig {
    private ExampleRobotCoreConfig() {}

    public static RobotCore.RobotCoreConfig<?> config(ConfigurableCamera... cameras) {
        // NOTE: This is intentionally minimal and may need additional drivetrain module configs
        // to be runnable on a real robot. It exists mainly to ensure the "happy path" API compiles.
        SwerveDrivetrainConfig drivetrain = SwerveDrivetrainConfig.standard(0.6)
                .setIMU(AthenaImu.PIGEON2, false)
                .setFieldRelative(true);

        RobotLocalizationConfig localization = RobotLocalizationConfig.vision(0.8, 0.8, 9999)
                .setAutoPlannerPID(7, 0, 0, 2, 0, 0)
                .setVisionEnabled(true)
                .setAutoPoseName("field");

        return RobotCore.RobotCoreConfig.<SwerveDrivetrain>swerve(drivetrain)
                .setLocalization(localization)
                .setVision(cameras)
                .setPerformanceMode(false)
                .setTelemetryEnabled(true);
    }
}
