package ca.frc6390.athena.core.sim;

import ca.frc6390.athena.core.RobotVision;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;

/**
 * Lightweight placeholder simulation bridge that keeps the core module free of vendor vision
 * dependencies. Vendor modules can extend/replace this behavior via their own providers.
 */
public class RobotVisionSim {
    protected final RobotVision vision;
    protected final AprilTagFieldLayout layout;

    public RobotVisionSim(RobotVision vision, AprilTagFieldLayout layout) {
        this.vision = vision;
        this.layout = layout;
    }

    public void update(Pose2d robotPose) {
        // no-op placeholder
    }
}
