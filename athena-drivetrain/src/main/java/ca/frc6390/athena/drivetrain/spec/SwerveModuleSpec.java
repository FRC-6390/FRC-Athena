package ca.frc6390.athena.drivetrain.spec;

import ca.frc6390.athena.hardware.encoder.EncoderSpec;
import ca.frc6390.athena.hardware.spec.MotorSpec;

/**
 * Immutable swerve module declaration.
 *
 * @param name module name
 * @param driveMotor drive motor spec
 * @param steerMotor steer motor spec
 * @param steerEncoder steer encoder spec
 * @param xMeters forward-positive location from robot center
 * @param yMeters left-positive location from robot center
 * @param driveInverted drive motor inversion
 * @param steerInverted steer motor inversion
 * @param encoderInverted steer encoder inversion
 * @param control module control gains
 */
public record SwerveModuleSpec(
        String name,
        MotorSpec driveMotor,
        MotorSpec steerMotor,
        EncoderSpec steerEncoder,
        double xMeters,
        double yMeters,
        boolean driveInverted,
        boolean steerInverted,
        boolean encoderInverted,
        SwerveModuleControlSpec control) {
    public SwerveModuleSpec {
        name = name == null || name.isBlank() ? "module" : name;
        control = control == null ? new SwerveModuleControlSpec(0.0, 0.0, 0.0, 0.0, 0.0, 0.0) : control;
    }

    /**
     * Returns a dotted path for validation errors.
     *
     * @param drivetrainName drivetrain name
     * @return module path
     */
    public String path(String drivetrainName) {
        return drivetrainName + "." + name;
    }
}
