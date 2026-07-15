package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.runtime.control.RobotVelocity;
import edu.wpi.first.math.geometry.Pose2d;
import java.util.Objects;

/** Backend-independent desired field pose and field-relative feedforward velocity. */
public record SwervePathSample(Pose2d pose, RobotVelocity fieldVelocity) {
    public SwervePathSample {
        Objects.requireNonNull(pose, "pose");
        fieldVelocity = fieldVelocity == null ? RobotVelocity.zero() : fieldVelocity;
    }
}
