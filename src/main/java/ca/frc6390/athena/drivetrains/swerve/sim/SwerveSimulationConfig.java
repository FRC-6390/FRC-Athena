package ca.frc6390.athena.drivetrains.swerve.sim;

public class SwerveSimulationConfig {

    // Use real robot numbers: weigh the robot for mass, estimate yaw inertia from CAD or m*(w^2+l^2)/12, pick a tire friction that matches practice carpet (~1.1-1.3), and leave voltage at nominal battery.
    private final double robotMassKg;
    private final double robotMomentOfInertia;
    private final double wheelCoefficientOfFriction;
    private final double nominalVoltage;

    private SwerveSimulationConfig(double robotMassKg, double robotMomentOfInertia, double wheelCoefficientOfFriction, double nominalVoltage) {
        this.robotMassKg = robotMassKg;
        this.robotMomentOfInertia = robotMomentOfInertia;
        this.wheelCoefficientOfFriction = wheelCoefficientOfFriction;
        this.nominalVoltage = nominalVoltage;
    }

    public static SwerveSimulationConfig defaults() {
        return new SwerveSimulationConfig(58.0, 6.0, 1.3, 12.0);
    }

    public SwerveSimulationConfig withRobotMassKg(double robotMassKg) {
        return new SwerveSimulationConfig(robotMassKg, robotMomentOfInertia, wheelCoefficientOfFriction, nominalVoltage);
    }

    public SwerveSimulationConfig withRobotMomentOfInertia(double robotMomentOfInertia) {
        return new SwerveSimulationConfig(robotMassKg, robotMomentOfInertia, wheelCoefficientOfFriction, nominalVoltage);
    }

    public SwerveSimulationConfig withWheelCoefficientOfFriction(double wheelCoefficientOfFriction) {
        return new SwerveSimulationConfig(robotMassKg, robotMomentOfInertia, wheelCoefficientOfFriction, nominalVoltage);
    }

    public SwerveSimulationConfig withNominalVoltage(double nominalVoltage) {
        return new SwerveSimulationConfig(robotMassKg, robotMomentOfInertia, wheelCoefficientOfFriction, nominalVoltage);
    }

    public double getRobotMassKg() {
        return robotMassKg;
    }

    public double getRobotMomentOfInertia() {
        return robotMomentOfInertia;
    }

    public double getWheelCoefficientOfFriction() {
        return wheelCoefficientOfFriction;
    }

    public double getNominalVoltage() {
        return nominalVoltage;
    }
}
