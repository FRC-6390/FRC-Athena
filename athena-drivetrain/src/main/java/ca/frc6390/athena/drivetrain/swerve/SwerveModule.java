package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismState;
import ca.frc6390.athena.mechanism.core.States;
import java.util.Objects;

/**
 * Base type for a swerve module declared as a specialized mechanism.
 */
public abstract class SwerveModule implements Mechanism {
    private final SwerveModuleModel model;
    private final MechanismState idle = States.neutral();
    private double targetSpeedMetersPerSecond;
    private double targetAngleRadians;

    protected SwerveModule(SwerveModuleModel model) {
        this.model = Objects.requireNonNull(model, "model");
    }

    public SwerveModuleModel model() {
        return model;
    }

    public double targetSpeedMetersPerSecond() {
        return targetSpeedMetersPerSecond;
    }

    public double targetAngleRadians() {
        return targetAngleRadians;
    }

    public void target(double speedMetersPerSecond, double angleRadians) {
        targetSpeedMetersPerSecond = speedMetersPerSecond;
        targetAngleRadians = angleRadians;
    }

    @Override
    public MechanismState initialState() {
        return idle;
    }
}
