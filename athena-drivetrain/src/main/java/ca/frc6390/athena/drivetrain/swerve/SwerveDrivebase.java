package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.MechanismState;
import ca.frc6390.athena.mechanism.core.States;

/**
 * Mechanism-style marker for an introspected swerve drivebase.
 */
public interface SwerveDrivebase extends Mechanism {
    /**
     * Drivebases are controlled by drive modes, not user-defined mechanism states.
     *
     * @return neutral internal state
     */
    @Override
    default MechanismState initialState() {
        return States.neutral();
    }
}
