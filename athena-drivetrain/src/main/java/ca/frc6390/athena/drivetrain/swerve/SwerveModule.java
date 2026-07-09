package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.State;
import ca.frc6390.athena.mechanism.core.States;
import java.util.Objects;

/**
 * Base type for a swerve module declared as a specialized mechanism.
 */
public abstract class SwerveModule implements Mechanism {
    private final SwerveModuleModel model;
    private final State idle = States.neutral();

    protected SwerveModule(SwerveModuleModel model) {
        this.model = Objects.requireNonNull(model, "model");
    }

    public SwerveModuleModel model() {
        return model;
    }

    @Override
    public State initialState() {
        return idle;
    }
}
