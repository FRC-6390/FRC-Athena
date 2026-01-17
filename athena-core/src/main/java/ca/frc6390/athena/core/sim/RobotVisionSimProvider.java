package ca.frc6390.athena.core.sim;

import ca.frc6390.athena.core.RobotVision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;

/**
 * Service provider used to supply a vision simulation implementation.
 */
public interface RobotVisionSimProvider {
    boolean supports(RobotVision vision);

    RobotVisionSim create(RobotVision vision, AprilTagFieldLayout layout);
}
