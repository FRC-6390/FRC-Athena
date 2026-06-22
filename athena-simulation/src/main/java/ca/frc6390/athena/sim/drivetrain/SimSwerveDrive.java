package ca.frc6390.athena.sim.drivetrain;

import java.util.Objects;

import ca.frc6390.athena.drivetrain.spec.SwerveDrivetrainSpec;
import ca.frc6390.athena.drivetrain.spec.SwerveModuleSpec;
import ca.frc6390.athena.sim.world.SimMotorState;
import ca.frc6390.athena.sim.world.SimWorld;

/**
 * Simulation binding for a swerve drivetrain spec.
 */
public final class SimSwerveDrive {
    private final SimWorld world;
    private final SwerveDrivetrainSpec spec;

    /**
     * Creates a simulated swerve drivetrain binding.
     *
     * @param world simulation world
     * @param spec swerve drivetrain spec
     */
    public SimSwerveDrive(SimWorld world, SwerveDrivetrainSpec spec) {
        this.world = Objects.requireNonNull(world, "world");
        this.spec = Objects.requireNonNull(spec, "spec");
    }

    /**
     * Applies the same velocity and steer angle to every module.
     *
     * @param velocityMetersPerSecond module drive velocity
     * @param angleDegrees module steer angle
     * @return this drivetrain binding
     */
    public SimSwerveDrive driveAll(double velocityMetersPerSecond, double angleDegrees) {
        spec.modules().forEach(module -> module(module.name(), velocityMetersPerSecond, angleDegrees));
        return this;
    }

    /**
     * Applies a module-level velocity and steer angle target.
     *
     * @param moduleName module name
     * @param velocityMetersPerSecond module drive velocity
     * @param angleDegrees steer angle target
     * @return this drivetrain binding
     */
    public SimSwerveDrive module(String moduleName, double velocityMetersPerSecond, double angleDegrees) {
        SwerveModuleSpec module = requireModule(moduleName);
        double velocity = Double.isFinite(velocityMetersPerSecond) ? velocityMetersPerSecond : 0.0;
        double angle = Double.isFinite(angleDegrees) ? angleDegrees : 0.0;
        world.motor(module.driveMotor().path()).velocityPerSecond(module.driveInverted() ? -velocity : velocity);
        world.motor(module.steerMotor().path()).position(module.steerInverted() ? -angle : angle).velocityPerSecond(0.0);
        return this;
    }

    /**
     * Returns the simulated drive motor state for a module.
     *
     * @param moduleName module name
     * @return drive motor state
     */
    public SimMotorState driveMotor(String moduleName) {
        return world.motor(requireModule(moduleName).driveMotor().path());
    }

    /**
     * Returns the simulated steer motor state for a module.
     *
     * @param moduleName module name
     * @return steer motor state
     */
    public SimMotorState steerMotor(String moduleName) {
        return world.motor(requireModule(moduleName).steerMotor().path());
    }

    private SwerveModuleSpec requireModule(String moduleName) {
        return spec.modules().stream()
                .filter(module -> module.name().equals(moduleName))
                .findFirst()
                .orElseThrow(() -> new IllegalArgumentException(
                        "Unknown module " + moduleName + " in swerve drivetrain " + spec.name() + "."));
    }
}
