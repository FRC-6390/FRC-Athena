package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.sim.SimModels;
import ca.frc6390.athena.hardware.sim.SimProfile;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/**
 * Simulation model factories for swerve modules.
 */
public final class SwerveSimModels {
    private SwerveSimModels() {
    }

    /**
     * Creates simulation models for a swerve module's drive and steer axes.
     *
     * @param module filled swerve module
     * @return drive and steer simulation models
     */
    public static List<SimModel> module(SwerveModule module) {
        Objects.requireNonNull(module, "module");
        if (!module.drive.filled() || !module.steer.filled() || !module.angle.filled()) {
            throw new IllegalStateException("Swerve module slots must be filled before creating simulation models.");
        }

        SwerveModuleModel model = module.model();
        SimModel drive = SimModels.motor(module.drive.get())
                .encoder(module.drive.get().encoder())
                .gearRatio(GearRatio.reduction(model.driveReduction(), 1.0));
        SimModel steer = SimModels.motor(module.steer.get())
                .encoder(module.angle.get())
                .gearRatio(GearRatio.reduction(model.steerReduction(), 1.0));
        return List.of(drive, steer);
    }

    /**
     * Creates simulation models for multiple filled swerve modules.
     *
     * @param modules filled swerve modules
     * @return simulation models
     */
    public static List<SimModel> modules(SwerveModule... modules) {
        if (modules == null) {
            return List.of();
        }
        return java.util.Arrays.stream(modules)
                .flatMap(module -> module(module).stream())
                .toList();
    }

    /**
     * Creates simulation models for a rectangular swerve drivetrain.
     *
     * @param wheelBaseMeters front-back wheel distance
     * @param trackWidthMeters left-right wheel distance
     * @param maxSpeedMetersPerSecond fallback open-loop speed
     * @param frontLeft front-left module
     * @param frontRight front-right module
     * @param backLeft back-left module
     * @param backRight back-right module
     * @return module models plus drivetrain pose model
     */
    public static List<SimModel> drive(
            double wheelBaseMeters,
            double trackWidthMeters,
            double maxSpeedMetersPerSecond,
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight) {
        List<SimModel> models = new ArrayList<>(modules(frontLeft, frontRight, backLeft, backRight));
        models.add(SimModel.of(SimProfile.Kind.MOTOR)
                .dependency(SwerveDriveSimModel.rectangular(
                        wheelBaseMeters,
                        trackWidthMeters,
                        frontLeft,
                        frontRight,
                        backLeft,
                        backRight,
                        maxSpeedMetersPerSecond)));
        return List.copyOf(models);
    }
}
