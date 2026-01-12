package ca.frc6390.athena.core.auto;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Vendor-agnostic facade over Choreo trajectory execution.
 */
public interface ChoreoAutoFactory {
    Command trajectoryCmd(String trajectoryName);

    Command resetOdometry(String trajectoryName);
}
