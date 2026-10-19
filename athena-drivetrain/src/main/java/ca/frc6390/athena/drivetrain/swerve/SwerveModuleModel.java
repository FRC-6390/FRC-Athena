package ca.frc6390.athena.drivetrain.swerve;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.units.measure.Distance;
import java.util.Objects;

/**
 * Known physical model for a swerve module.
 *
 * @param vendor module vendor/family
 * @param name module name
 * @param driveReduction motor-to-wheel reduction
 * @param steerReduction steer motor-to-azimuth reduction
 * @param wheelDiameter wheel diameter
 */
public record SwerveModuleModel(
        String vendor,
        String name,
        double driveReduction,
        double steerReduction,
        Distance wheelDiameter) {
    public SwerveModuleModel {
        vendor = vendor == null || vendor.isBlank() ? "custom" : vendor;
        name = name == null || name.isBlank() ? "custom" : name;
        requirePositive(driveReduction, "driveReduction");
        requirePositive(steerReduction, "steerReduction");
        Objects.requireNonNull(wheelDiameter, "wheelDiameter");
        requirePositive(wheelDiameter.in(Meters), "wheelDiameter");
    }

    public static SwerveModuleModel custom(double driveReduction, double steerReduction, Distance wheelDiameter) {
        return new SwerveModuleModel("custom", "custom", driveReduction, steerReduction, wheelDiameter);
    }

    public SwerveModuleModel withDriveReduction(double driveReduction) {
        return new SwerveModuleModel(vendor, name, driveReduction, steerReduction, wheelDiameter);
    }

    public SwerveModuleModel withSteerReduction(double steerReduction) {
        return new SwerveModuleModel(vendor, name, driveReduction, steerReduction, wheelDiameter);
    }

    public SwerveModuleModel withWheelDiameter(Distance wheelDiameter) {
        return new SwerveModuleModel(vendor, name, driveReduction, steerReduction, wheelDiameter);
    }

    /** Wheel circumference, the drive travel per motor-side revolution before reduction. */
    Distance wheelCircumference() {
        return Meters.of(Math.PI * wheelDiameter.in(Meters));
    }

    private static void requirePositive(double value, String name) {
        Objects.requireNonNull(name, "name");
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(name + " must be positive.");
        }
    }
}
