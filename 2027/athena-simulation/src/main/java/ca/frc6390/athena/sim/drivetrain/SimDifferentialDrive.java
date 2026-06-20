package ca.frc6390.athena.sim.drivetrain;

import java.util.Objects;

import ca.frc6390.athena.drivetrain.spec.DifferentialDrivetrainSpec;
import ca.frc6390.athena.hardware.spec.MotorSpec;
import ca.frc6390.athena.sim.world.SimMotorState;
import ca.frc6390.athena.sim.world.SimWorld;

/**
 * Simulation binding for a differential drivetrain spec.
 */
public final class SimDifferentialDrive {
    private final SimWorld world;
    private final DifferentialDrivetrainSpec spec;

    /**
     * Creates a simulated differential drivetrain binding.
     *
     * @param world simulation world
     * @param spec differential drivetrain spec
     */
    public SimDifferentialDrive(SimWorld world, DifferentialDrivetrainSpec spec) {
        this.world = Objects.requireNonNull(world, "world");
        this.spec = Objects.requireNonNull(spec, "spec");
    }

    /**
     * Applies side velocity commands to every declared side motor.
     *
     * @param leftVelocityPerSecond left side velocity
     * @param rightVelocityPerSecond right side velocity
     * @return this drivetrain binding
     */
    public SimDifferentialDrive tankVelocity(double leftVelocityPerSecond, double rightVelocityPerSecond) {
        double safeLeft = finiteOrZero(leftVelocityPerSecond);
        double safeRight = finiteOrZero(rightVelocityPerSecond);
        spec.leftMotors().forEach(motor -> world.motor(motor.path()).velocityPerSecond(safeLeft));
        spec.rightMotors().forEach(motor -> world.motor(motor.path()).velocityPerSecond(safeRight));
        return this;
    }

    /**
     * Returns a simulated motor state by motor name.
     *
     * @param motorName motor name inside the drivetrain
     * @return simulated motor state
     */
    public SimMotorState motor(String motorName) {
        return spec.leftMotors().stream()
                .filter(motor -> motor.name().equals(motorName))
                .findFirst()
                .or(() -> spec.rightMotors().stream().filter(motor -> motor.name().equals(motorName)).findFirst())
                .map(MotorSpec::path)
                .map(world::motor)
                .orElseThrow(() -> new IllegalArgumentException(
                        "Unknown motor " + motorName + " in drivetrain " + spec.name() + "."));
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
