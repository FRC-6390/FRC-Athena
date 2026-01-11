package ca.frc6390.athena.drivetrains.differential.sim;

import ca.frc6390.athena.sim.Motor;
import ca.frc6390.athena.sim.MotorSimType;

/**
 * Immutable configuration describing the physical parameters required to simulate a differential
 * drivetrain. Teams can start from {@link #defaults()} and selectively override fields, or build a
 * bespoke configuration using the fluent {@code with*} helpers. Any unset geometry parameters can
 * be filled using {@link #resolve(double, double, double, int)} based on the drivetrain builder
 * inputs.
 */
public class DifferentialSimulationConfig {

    private final MotorSimType motorType;
    private final int motorsPerSide;
    private final double gearRatio;
    private final double wheelDiameterMeters;
    private final double trackWidthMeters;
    private final double robotMassKg;
    private final double robotMomentOfInertia;
    private final double nominalVoltage;

    private DifferentialSimulationConfig(
            MotorSimType motorType,
            int motorsPerSide,
            double gearRatio,
            double wheelDiameterMeters,
            double trackWidthMeters,
            double robotMassKg,
            double robotMomentOfInertia,
            double nominalVoltage) {
        this.motorType = motorType;
        this.motorsPerSide = motorsPerSide;
        this.gearRatio = gearRatio;
        this.wheelDiameterMeters = wheelDiameterMeters;
        this.trackWidthMeters = trackWidthMeters;
        this.robotMassKg = robotMassKg;
        this.robotMomentOfInertia = robotMomentOfInertia;
        this.nominalVoltage = nominalVoltage;
    }

    /**
     * Produces a baseline configuration that mirrors the team's reference kitbot. Values can be
     * overridden using the {@code with*} helpers prior to resolving against the drivetrain config.
     */
    public static DifferentialSimulationConfig defaults() {
        return new DifferentialSimulationConfig(
                Motor.CIM,
                -1,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN,
                Double.NaN);
    }

    public DifferentialSimulationConfig withMotor(MotorSimType motorType) {
        return new DifferentialSimulationConfig(
                motorType,
                motorsPerSide,
                gearRatio,
                wheelDiameterMeters,
                trackWidthMeters,
                robotMassKg,
                robotMomentOfInertia,
                nominalVoltage);
    }

    public DifferentialSimulationConfig withMotorsPerSide(int motorsPerSide) {
        return new DifferentialSimulationConfig(
                motorType,
                motorsPerSide,
                gearRatio,
                wheelDiameterMeters,
                trackWidthMeters,
                robotMassKg,
                robotMomentOfInertia,
                nominalVoltage);
    }

    public DifferentialSimulationConfig withGearRatio(double gearRatio) {
        return new DifferentialSimulationConfig(
                motorType,
                motorsPerSide,
                gearRatio,
                wheelDiameterMeters,
                trackWidthMeters,
                robotMassKg,
                robotMomentOfInertia,
                nominalVoltage);
    }

    public DifferentialSimulationConfig withWheelDiameterMeters(double wheelDiameterMeters) {
        return new DifferentialSimulationConfig(
                motorType,
                motorsPerSide,
                gearRatio,
                wheelDiameterMeters,
                trackWidthMeters,
                robotMassKg,
                robotMomentOfInertia,
                nominalVoltage);
    }

    public DifferentialSimulationConfig withTrackWidthMeters(double trackWidthMeters) {
        return new DifferentialSimulationConfig(
                motorType,
                motorsPerSide,
                gearRatio,
                wheelDiameterMeters,
                trackWidthMeters,
                robotMassKg,
                robotMomentOfInertia,
                nominalVoltage);
    }

    public DifferentialSimulationConfig withRobotMassKg(double robotMassKg) {
        return new DifferentialSimulationConfig(
                motorType,
                motorsPerSide,
                gearRatio,
                wheelDiameterMeters,
                trackWidthMeters,
                robotMassKg,
                robotMomentOfInertia,
                nominalVoltage);
    }

    public DifferentialSimulationConfig withRobotMomentOfInertia(double robotMomentOfInertia) {
        return new DifferentialSimulationConfig(
                motorType,
                motorsPerSide,
                gearRatio,
                wheelDiameterMeters,
                trackWidthMeters,
                robotMassKg,
                robotMomentOfInertia,
                nominalVoltage);
    }

    public DifferentialSimulationConfig withNominalVoltage(double nominalVoltage) {
        return new DifferentialSimulationConfig(
                motorType,
                motorsPerSide,
                gearRatio,
                wheelDiameterMeters,
                trackWidthMeters,
                robotMassKg,
                robotMomentOfInertia,
                nominalVoltage);
    }

    /**
     * Resolves any unset fields using the drivetrain's physical measurements or conservative
     * defaults. This should be invoked immediately before constructing the simulation so that wheel
     * geometry stays aligned with the runtime configuration.
     *
     * @param fallbackTrackWidthMeters drivetrain width from the configuration builder
     * @param fallbackGearRatio gearbox reduction from motor to wheel (motor rotations per wheel rotation)
     * @param fallbackWheelDiameterMeters physical wheel diameter used for odometry conversions
     * @param fallbackMotorsPerSide number of motors attached to each drivetrain side
     * @return resolved configuration with all parameters populated
     */
    public DifferentialSimulationConfig resolve(double fallbackTrackWidthMeters,
                                                double fallbackGearRatio,
                                                double fallbackWheelDiameterMeters,
                                                int fallbackMotorsPerSide) {
        MotorSimType resolvedMotor = motorType != null ? motorType : Motor.CIM;

        double resolvedGearRatio = Double.isFinite(gearRatio) && gearRatio > 0
                ? gearRatio
                : (Double.isFinite(fallbackGearRatio) && fallbackGearRatio > 0 ? fallbackGearRatio : 6.75);

        double resolvedWheelDiameter = Double.isFinite(wheelDiameterMeters) && wheelDiameterMeters > 0
                ? wheelDiameterMeters
                : (Double.isFinite(fallbackWheelDiameterMeters) && fallbackWheelDiameterMeters > 0
                        ? fallbackWheelDiameterMeters
                        : 0.1524);

        double resolvedTrackWidth = Double.isFinite(trackWidthMeters) && trackWidthMeters > 0
                ? trackWidthMeters
                : (Double.isFinite(fallbackTrackWidthMeters) && fallbackTrackWidthMeters > 0
                        ? fallbackTrackWidthMeters
                        : 0.6);

        double resolvedMass = Double.isFinite(robotMassKg) && robotMassKg > 0
                ? robotMassKg
                : 58.0;

        double resolvedMoi = Double.isFinite(robotMomentOfInertia) && robotMomentOfInertia > 0
                ? robotMomentOfInertia
                : 6.0;

        double resolvedVoltage = Double.isFinite(nominalVoltage) && nominalVoltage > 0
                ? nominalVoltage
                : 12.0;

        int resolvedMotorsPerSide = motorsPerSide > 0
                ? motorsPerSide
                : Math.max(1, fallbackMotorsPerSide);

        return new DifferentialSimulationConfig(
                resolvedMotor,
                resolvedMotorsPerSide,
                resolvedGearRatio,
                resolvedWheelDiameter,
                resolvedTrackWidth,
                resolvedMass,
                resolvedMoi,
                resolvedVoltage);
    }

    public MotorSimType getMotorType() {
        return motorType;
    }

    public int getMotorsPerSide() {
        return motorsPerSide;
    }

    public double getGearRatio() {
        return gearRatio;
    }

    public double getWheelDiameterMeters() {
        return wheelDiameterMeters;
    }

    public double getTrackWidthMeters() {
        return trackWidthMeters;
    }

    public double getRobotMassKg() {
        return robotMassKg;
    }

    public double getRobotMomentOfInertia() {
        return robotMomentOfInertia;
    }

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    /**
     * Convenience helper returning the wheel radius derived from the configured diameter.
     */
    public double getWheelRadiusMeters() {
        return wheelDiameterMeters / 2.0;
    }
}
