package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.drivetrain.spec.TrackWidth;
import ca.frc6390.athena.drivetrain.spec.WheelBase;
import ca.frc6390.athena.hardware.ref.ImuRef;
import java.util.List;

/**
 * Introspected swerve drivebase structure.
 *
 * @param drivebase drivebase instance
 * @param trackWidth track width, if declared
 * @param wheelBase wheelbase, if declared
 * @param imu IMU ref, if declared
 * @param modules discovered modules in runtime order
 */
public record SwerveDrivebaseDefinition(
        SwerveDrivebase drivebase,
        TrackWidth trackWidth,
        WheelBase wheelBase,
        ImuRef imu,
        List<SwerveModuleDefinition> modules) {
    public SwerveDrivebaseDefinition {
        modules = modules == null ? List.of() : List.copyOf(modules);
    }
}
