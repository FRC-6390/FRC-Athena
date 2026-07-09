package ca.frc6390.athena.drivetrain.swerve;

import java.util.List;
import java.util.Objects;

/**
 * Athena-owned simulation descriptor for a swerve drivetrain.
 *
 * @param modules module locations relative to robot center
 * @param maxSpeedMetersPerSecond fallback speed used for open-loop drive commands
 */
public record SwerveDriveSimModel(List<Module> modules, double maxSpeedMetersPerSecond) {
    public SwerveDriveSimModel {
        modules = modules == null ? List.of() : List.copyOf(modules);
        if (modules.isEmpty()) {
            throw new IllegalArgumentException("At least one swerve module is required.");
        }
        if (!Double.isFinite(maxSpeedMetersPerSecond) || maxSpeedMetersPerSecond <= 0.0) {
            throw new IllegalArgumentException("Max speed must be positive.");
        }
        modules.forEach(Objects::requireNonNull);
    }

    /**
     * Creates a four-module rectangular drivetrain descriptor.
     *
     * @param wheelBaseMeters front-back wheel distance
     * @param trackWidthMeters left-right wheel distance
     * @param frontLeft front-left module
     * @param frontRight front-right module
     * @param backLeft back-left module
     * @param backRight back-right module
     * @param maxSpeedMetersPerSecond fallback open-loop speed
     * @return drivetrain simulation descriptor
     */
    public static SwerveDriveSimModel rectangular(
            double wheelBaseMeters,
            double trackWidthMeters,
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight,
            double maxSpeedMetersPerSecond) {
        if (!Double.isFinite(wheelBaseMeters) || wheelBaseMeters <= 0.0) {
            throw new IllegalArgumentException("Wheelbase must be positive.");
        }
        if (!Double.isFinite(trackWidthMeters) || trackWidthMeters <= 0.0) {
            throw new IllegalArgumentException("Track width must be positive.");
        }
        double x = wheelBaseMeters / 2.0;
        double y = trackWidthMeters / 2.0;
        return new SwerveDriveSimModel(List.of(
                new Module(frontLeft, x, y),
                new Module(frontRight, x, -y),
                new Module(backLeft, -x, y),
                new Module(backRight, -x, -y)), maxSpeedMetersPerSecond);
    }

    /**
     * Swerve module position relative to robot center.
     *
     * @param module filled module
     * @param xMeters forward position
     * @param yMeters left position
     */
    public record Module(SwerveModule module, double xMeters, double yMeters) {
        public Module {
            Objects.requireNonNull(module, "module");
            if (!module.drive.filled() || !module.steer.filled() || !module.angle.filled()) {
                throw new IllegalArgumentException("Swerve module slots must be filled before simulation.");
            }
            if (!Double.isFinite(xMeters) || !Double.isFinite(yMeters)) {
                throw new IllegalArgumentException("Module position must be finite.");
            }
        }
    }
}
