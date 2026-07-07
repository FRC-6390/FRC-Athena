package ca.frc6390.athena.drivetrain.swerve;

import java.util.Objects;

/**
 * Known physical model for a swerve module.
 *
 * @param vendor module vendor/family
 * @param name module name
 * @param driveReduction motor-to-wheel reduction
 * @param steerReduction steer motor-to-azimuth reduction
 * @param wheelDiameterMeters wheel diameter
 */
public record SwerveModuleModel(
        String vendor,
        String name,
        double driveReduction,
        double steerReduction,
        double wheelDiameterMeters) {
    public SwerveModuleModel {
        vendor = vendor == null || vendor.isBlank() ? "custom" : vendor;
        name = name == null || name.isBlank() ? "custom" : name;
        requirePositive(driveReduction, "driveReduction");
        requirePositive(steerReduction, "steerReduction");
        requirePositive(wheelDiameterMeters, "wheelDiameterMeters");
    }

    public static SwerveModuleModel custom(double driveReduction, double steerReduction, double wheelDiameterMeters) {
        return new SwerveModuleModel("custom", "custom", driveReduction, steerReduction, wheelDiameterMeters);
    }

    public SwerveModuleModel withDriveReduction(double driveReduction) {
        return new SwerveModuleModel(vendor, name, driveReduction, steerReduction, wheelDiameterMeters);
    }

    public SwerveModuleModel withSteerReduction(double steerReduction) {
        return new SwerveModuleModel(vendor, name, driveReduction, steerReduction, wheelDiameterMeters);
    }

    public SwerveModuleModel withWheelDiameterMeters(double wheelDiameterMeters) {
        return new SwerveModuleModel(vendor, name, driveReduction, steerReduction, wheelDiameterMeters);
    }

    private static void requirePositive(double value, String name) {
        Objects.requireNonNull(name, "name");
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be positive.");
        }
    }
}
