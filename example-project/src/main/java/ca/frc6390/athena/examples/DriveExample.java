package ca.frc6390.athena.examples;

import ca.frc6390.athena.drivetrain.config.DifferentialDrivetrainConfig;
import ca.frc6390.athena.drivetrain.config.Drivetrains;
import ca.frc6390.athena.drivetrain.spec.TrackWidth;

/**
 * Differential drivetrain example using generic motor declarations.
 */
public final class DriveExample {
    /**
     * Differential drivetrain declaration.
     */
    public static final DifferentialDrivetrainConfig CONFIG = Drivetrains.differential("drive")
            .leftMotor("leftLeader", motor -> motor
                    .hardware(RobotHardware.DRIVE_LEFT_LEADER)
                    .brake()
                    .currentLimit(45))
            .rightMotor("rightLeader", motor -> motor
                    .hardware(RobotHardware.DRIVE_RIGHT_LEADER)
                    .brake()
                    .currentLimit(45))
            .trackWidth(TrackWidth.meters(0.71));

    private DriveExample() {
    }
}
