package ca.frc6390.athena.commands.vision;

import java.util.Objects;
import java.util.OptionalDouble;

import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.sensors.camera.LocalizationCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Minimal command that rotates the robot until the selected camera reports the tag centered in view.
 */
public class RotateToTagCommand extends Command {

    private final RobotSpeeds robotSpeeds;
    private final LocalizationCamera camera;
    private final PIDController headingController;
    private final double headingOffsetDegrees;
    private final double maxAngularVelocityRadPerSec;
    private final double timeoutSeconds;

    private double startTime;

    /**
     * Creates a new rotate-to-tag command.
     *
     * @param robotSpeeds active robot speed aggregator
     * @param camera camera used to supply tag yaw measurements
     * @param headingController PID controller configured for heading correction (degrees domain)
     * @param headingOffsetDegrees optional offset in degrees to apply to the tag heading
     * @param maxAngularVelocityRadPerSec clamp for the angular velocity command
     * @param timeoutSeconds command timeout in seconds ({@code <= 0} disables the timeout)
     */
    public RotateToTagCommand(
            RobotSpeeds robotSpeeds,
            LocalizationCamera camera,
            PIDController headingController,
            double headingOffsetDegrees,
            double maxAngularVelocityRadPerSec,
            double timeoutSeconds) {
        this.robotSpeeds = Objects.requireNonNull(robotSpeeds, "robotSpeeds");
        this.camera = Objects.requireNonNull(camera, "camera");
        this.headingController = Objects.requireNonNull(headingController, "headingController");
        this.headingOffsetDegrees = headingOffsetDegrees;
        this.maxAngularVelocityRadPerSec = Math.max(0.0, maxAngularVelocityRadPerSec);
        this.timeoutSeconds = timeoutSeconds;
    }

    /**
     * Convenience constructor with zero offset and no timeout.
     */
    public RotateToTagCommand(
            RobotSpeeds robotSpeeds,
            LocalizationCamera camera,
            PIDController headingController,
            double maxAngularVelocityRadPerSec) {
        this(robotSpeeds, camera, headingController, 0.0, maxAngularVelocityRadPerSec, 0.0);
    }

    @Override
    public void initialize() {
        headingController.reset();
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        OptionalDouble yawDegrees = camera.getTargetYawDegrees();
        if (yawDegrees.isEmpty()) {
            robotSpeeds.stopSpeeds("feedback");
            return;
        }

        double errorDegrees = yawDegrees.getAsDouble();
        double controllerOutput =
                headingController.calculate(errorDegrees, headingOffsetDegrees);
        double omega =
                -Math.toRadians(controllerOutput); // negate so positive yaw rotates back toward centre
        double clampedOmega = MathUtil.clamp(omega, -maxAngularVelocityRadPerSec, maxAngularVelocityRadPerSec);

        robotSpeeds.setSpeeds("feedback", 0.0, 0.0, clampedOmega);
    }

    @Override
    public void end(boolean interrupted) {
        robotSpeeds.stopSpeeds("feedback");
    }

    @Override
    public boolean isFinished() {
        boolean onTarget = headingController.atSetpoint();
        if (onTarget) {
            return true;
        }
        if (timeoutSeconds <= 0.0) {
            return false;
        }
        double elapsed = Timer.getFPGATimestamp() - startTime;
        return elapsed >= timeoutSeconds;
    }
}
