package ca.frc6390.athena.drivetrains.swerve.sim;

import ca.frc6390.athena.drivetrains.sim.CommonDrivetrainSimulationConfig;

/**
 * Immutable container holding the physical parameters required to approximate the drivetrain in
 * simulation. Use the provided {@code with*} methods to derive tweaked copies.
 */
public class SwerveSimulationConfig implements CommonDrivetrainSimulationConfig<SwerveSimulationConfig> {

    // Use real robot numbers: weigh the robot for mass, estimate yaw inertia from CAD or m*(w^2+l^2)/12, pick a tire friction that matches practice carpet (~1.1-1.3), and leave voltage at nominal battery.
    private final double robotMassKg;
    private final double robotMomentOfInertia;
    private final double wheelCoefficientOfFriction;
    private final double nominalVoltage;
    private final double maxSpeedScale;

    private SwerveSimulationConfig(double robotMassKg,
                                   double robotMomentOfInertia,
                                   double wheelCoefficientOfFriction,
                                   double nominalVoltage,
                                   double maxSpeedScale) {
        this.robotMassKg = robotMassKg;
        this.robotMomentOfInertia = robotMomentOfInertia;
        this.wheelCoefficientOfFriction = wheelCoefficientOfFriction;
        this.nominalVoltage = nominalVoltage;
        this.maxSpeedScale = maxSpeedScale;
    }

    /**
     * Returns a baseline configuration that matches the team's reference robot.
     */
    public static SwerveSimulationConfig defaults() {
        // Defaults tuned for more realistic accel in sim; override per-robot with actual values.
        return new SwerveSimulationConfig(58.0, 6.0, 0.7, 10.5, 1.0);
    }

    /**
     * Returns a copy of this configuration with an updated robot mass.
     *
     * @param robotMassKg chassis mass in kilograms
     */
    public SwerveSimulationConfig withRobotMassKg(double robotMassKg) {
        return new SwerveSimulationConfig(robotMassKg, robotMomentOfInertia, wheelCoefficientOfFriction, nominalVoltage, maxSpeedScale);
    }

    /**
     * Returns a copy of this configuration with an updated yaw moment of inertia.
     *
     * @param robotMomentOfInertia yaw inertia about the vertical axis (kg·m²)
     */
    public SwerveSimulationConfig withRobotMomentOfInertia(double robotMomentOfInertia) {
        return new SwerveSimulationConfig(robotMassKg, robotMomentOfInertia, wheelCoefficientOfFriction, nominalVoltage, maxSpeedScale);
    }

    /**
     * Returns a copy of this configuration with a new wheel friction coefficient.
     *
     * @param wheelCoefficientOfFriction friction coefficient to apply to each module
     */
    public SwerveSimulationConfig withWheelCoefficientOfFriction(double wheelCoefficientOfFriction) {
        return new SwerveSimulationConfig(robotMassKg, robotMomentOfInertia, wheelCoefficientOfFriction, nominalVoltage, maxSpeedScale);
    }

    /**
     * Returns a copy of this configuration with a new nominal voltage clamp.
     *
     * @param nominalVoltage battery voltage used to clamp drive commands
     */
    public SwerveSimulationConfig withNominalVoltage(double nominalVoltage) {
        return new SwerveSimulationConfig(robotMassKg, robotMomentOfInertia, wheelCoefficientOfFriction, nominalVoltage, maxSpeedScale);
    }

    /**
     * Returns a copy of this configuration with a scalar applied to the simulated maximum speed
     * (useful when sim feels too fast vs. real hardware).
     *
     * @param maxSpeedScale scale factor applied to linear/rotational limits (1.0 = unchanged)
     */
    public SwerveSimulationConfig withMaxSpeedScale(double maxSpeedScale) {
        return new SwerveSimulationConfig(robotMassKg, robotMomentOfInertia, wheelCoefficientOfFriction, nominalVoltage, maxSpeedScale);
    }

    /** Chassis mass in kilograms. */
    public double getRobotMassKg() {
        return robotMassKg;
    }

    /** Yaw moment of inertia (kg·m²). */
    public double getRobotMomentOfInertia() {
        return robotMomentOfInertia;
    }

    /** Estimated friction coefficient between the wheels and floor. */
    public double getWheelCoefficientOfFriction() {
        return wheelCoefficientOfFriction;
    }

    /** Nominal battery voltage used to clamp simulated motor commands. */
    public double getNominalVoltage() {
        return nominalVoltage;
    }

    /** Scale factor applied to simulated max linear/rotational speed limits. */
    public double getMaxSpeedScale() {
        return maxSpeedScale;
    }
}
