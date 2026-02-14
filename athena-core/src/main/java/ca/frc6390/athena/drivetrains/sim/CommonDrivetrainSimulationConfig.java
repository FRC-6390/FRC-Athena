package ca.frc6390.athena.drivetrains.sim;

/**
 * Shared mutable-copy contract for drivetrain simulation configs.
 */
public interface CommonDrivetrainSimulationConfig<T> {
    T withRobotMassKg(double robotMassKg);

    T withNominalVoltage(double nominalVoltage);

    T withRobotMomentOfInertia(double robotMomentOfInertia);
}
