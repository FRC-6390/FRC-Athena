package ca.frc6390.athena.examples;

import ca.frc6390.athena.commands.CommandSpec;
import ca.frc6390.athena.commands.RobotDriveCommands;
import ca.frc6390.athena.runtime.control.RobotSpeeds;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.vision.runtime.VisionTurnAssist;
import ca.frc6390.athena.vision.spec.VisionFrame;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Example drive, movement, and vision-assist command declarations.
 */
public final class DriveCommandExample {
    private DriveCommandExample() {
    }

    /**
     * Creates the shared speed blender used by the drive commands.
     *
     * @return robot speed blender
     */
    public static RobotSpeeds speeds() {
        return new RobotSpeeds(4.5, 3.0);
    }

    /**
     * Creates a tank-drive command from left and right velocity suppliers.
     *
     * @param speeds speed blender
     * @param leftMetersPerSecond left velocity
     * @param rightMetersPerSecond right velocity
     * @return command spec
     */
    public static CommandSpec tankDrive(
            RobotSpeeds speeds,
            DoubleSupplier leftMetersPerSecond,
            DoubleSupplier rightMetersPerSecond) {
        return RobotDriveCommands.tankDrive(speeds, leftMetersPerSecond, rightMetersPerSecond, 0.62);
    }

    /**
     * Creates a field-relative swerve-style drive command.
     *
     * @param speeds speed blender
     * @param xMetersPerSecond field-forward velocity
     * @param yMetersPerSecond field-left velocity
     * @param angularRadiansPerSecond angular velocity
     * @return command spec
     */
    public static CommandSpec fieldRelativeDrive(
            RobotSpeeds speeds,
            DoubleSupplier xMetersPerSecond,
            DoubleSupplier yMetersPerSecond,
            DoubleSupplier angularRadiansPerSecond) {
        return RobotDriveCommands.fieldRelativeDrive(
                speeds, xMetersPerSecond, yMetersPerSecond, angularRadiansPerSecond);
    }

    /**
     * Creates an autonomous distance command.
     *
     * @param speeds speed blender
     * @param measuredMeters measured travel distance
     * @return command spec
     */
    public static CommandSpec driveToLine(RobotSpeeds speeds, DoubleSupplier measuredMeters) {
        return RobotDriveCommands.driveDistance(speeds, measuredMeters, 2.4, 1.2);
    }

    /**
     * Creates a field-relative waypoint-following command.
     *
     * @param speeds speed blender
     * @param currentPose current robot pose
     * @return command spec
     */
    public static CommandSpec followSimplePath(RobotSpeeds speeds, Supplier<PoseSnapshot> currentPose) {
        return RobotDriveCommands.followWaypoints(
                speeds,
                currentPose,
                List.of(
                        new PoseSnapshot(1.5, 0.0, 0.0),
                        new PoseSnapshot(1.5, 1.0, Math.PI / 2.0)),
                1.2,
                2.0,
                0.1);
    }

    /**
     * Creates a target-alignment feedback command.
     *
     * @param speeds speed blender
     * @param targetVisible whether a vision target is available
     * @param yawErrorRadians yaw error from the target
     * @return command spec
     */
    public static CommandSpec alignToTarget(
            RobotSpeeds speeds,
            BooleanSupplier targetVisible,
            DoubleSupplier yawErrorRadians) {
        return RobotDriveCommands.visionTurnAssist(speeds, targetVisible, yawErrorRadians, -2.5, 0.03);
    }

    /**
     * Creates target-alignment feedback from generic camera frames.
     *
     * @param speeds speed blender
     * @param frames camera frame supplier
     * @return command spec
     */
    public static CommandSpec alignToVisionFrame(RobotSpeeds speeds, Supplier<VisionFrame> frames) {
        VisionTurnAssist assist = new VisionTurnAssist(speeds, frames, -2.5, 0.03);
        return CommandSpec.create("alignToVisionFrame")
                .onExecute(assist::execute)
                .until(assist::isAligned)
                .onEnd(assist::stop)
                .toSpec();
    }
}
