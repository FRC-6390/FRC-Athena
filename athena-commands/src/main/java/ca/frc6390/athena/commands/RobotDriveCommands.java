package ca.frc6390.athena.commands;

import ca.frc6390.athena.runtime.control.RobotSpeeds;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import java.util.List;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Dependency-free command factories for writing drive and feedback requests into {@link RobotSpeeds}.
 */
public final class RobotDriveCommands {
    private RobotDriveCommands() {
    }

    /**
     * Creates a tank-drive command.
     *
     * @param speeds robot speed blender
     * @param leftMetersPerSecond left side velocity supplier
     * @param rightMetersPerSecond right side velocity supplier
     * @param trackWidthMeters drivetrain track width
     * @return command spec
     */
    public static CommandSpec tankDrive(
            RobotSpeeds speeds,
            DoubleSupplier leftMetersPerSecond,
            DoubleSupplier rightMetersPerSecond,
            double trackWidthMeters) {
        Objects.requireNonNull(speeds, "speeds");
        Objects.requireNonNull(leftMetersPerSecond, "leftMetersPerSecond");
        Objects.requireNonNull(rightMetersPerSecond, "rightMetersPerSecond");
        double safeTrackWidth = finitePositive(trackWidthMeters, 1.0);
        return CommandSpec.create("tankDrive")
                .onExecute(() -> {
                    double left = finiteOrZero(leftMetersPerSecond.getAsDouble());
                    double right = finiteOrZero(rightMetersPerSecond.getAsDouble());
                    speeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, (left + right) / 2.0, 0.0, (right - left) / safeTrackWidth);
                })
                .onEnd(() -> speeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 0.0, 0.0, 0.0))
                .toSpec();
    }

    /**
     * Creates an arcade-drive command.
     *
     * @param speeds robot speed blender
     * @param forwardMetersPerSecond forward velocity supplier
     * @param angularRadiansPerSecond angular velocity supplier
     * @return command spec
     */
    public static CommandSpec arcadeDrive(
            RobotSpeeds speeds,
            DoubleSupplier forwardMetersPerSecond,
            DoubleSupplier angularRadiansPerSecond) {
        Objects.requireNonNull(speeds, "speeds");
        Objects.requireNonNull(forwardMetersPerSecond, "forwardMetersPerSecond");
        Objects.requireNonNull(angularRadiansPerSecond, "angularRadiansPerSecond");
        return CommandSpec.create("arcadeDrive")
                .onExecute(() -> speeds.setSpeeds(
                        RobotSpeeds.DRIVE_SOURCE,
                        finiteOrZero(forwardMetersPerSecond.getAsDouble()),
                        0.0,
                        finiteOrZero(angularRadiansPerSecond.getAsDouble())))
                .onEnd(() -> speeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 0.0, 0.0, 0.0))
                .toSpec();
    }

    /**
     * Creates a field-relative drive command.
     *
     * @param speeds robot speed blender
     * @param xMetersPerSecond field-forward velocity supplier
     * @param yMetersPerSecond field-left velocity supplier
     * @param angularRadiansPerSecond angular velocity supplier
     * @return command spec
     */
    public static CommandSpec fieldRelativeDrive(
            RobotSpeeds speeds,
            DoubleSupplier xMetersPerSecond,
            DoubleSupplier yMetersPerSecond,
            DoubleSupplier angularRadiansPerSecond) {
        Objects.requireNonNull(speeds, "speeds");
        Objects.requireNonNull(xMetersPerSecond, "xMetersPerSecond");
        Objects.requireNonNull(yMetersPerSecond, "yMetersPerSecond");
        Objects.requireNonNull(angularRadiansPerSecond, "angularRadiansPerSecond");
        return CommandSpec.create("fieldRelativeDrive")
                .onExecute(() -> speeds.setFieldRelativeSpeeds(
                        RobotSpeeds.DRIVE_SOURCE,
                        finiteOrZero(xMetersPerSecond.getAsDouble()),
                        finiteOrZero(yMetersPerSecond.getAsDouble()),
                        finiteOrZero(angularRadiansPerSecond.getAsDouble())))
                .onEnd(() -> speeds.setSpeeds(RobotSpeeds.DRIVE_SOURCE, 0.0, 0.0, 0.0))
                .toSpec();
    }

    /**
     * Creates a distance command that drives until measured travel reaches the target.
     *
     * @param speeds robot speed blender
     * @param measuredMeters measured distance supplier
     * @param targetMeters target travel distance
     * @param cruiseMetersPerSecond drive speed while unfinished
     * @return command spec
     */
    public static CommandSpec driveDistance(
            RobotSpeeds speeds,
            DoubleSupplier measuredMeters,
            double targetMeters,
            double cruiseMetersPerSecond) {
        Objects.requireNonNull(speeds, "speeds");
        Objects.requireNonNull(measuredMeters, "measuredMeters");
        double target = finiteOrZero(targetMeters);
        double cruise = finiteOrZero(cruiseMetersPerSecond);
        double direction = target < 0.0 ? -1.0 : 1.0;
        BooleanSupplier finished = () -> Math.abs(finiteOrZero(measuredMeters.getAsDouble())) >= Math.abs(target);
        return CommandSpec.create("driveDistance")
                .onExecute(() -> {
                    double velocity = finished.getAsBoolean() ? 0.0 : Math.abs(cruise) * direction;
                    speeds.setSpeeds(RobotSpeeds.AUTO_SOURCE, velocity, 0.0, 0.0);
                })
                .until(finished::getAsBoolean)
                .onEnd(() -> speeds.setSpeeds(RobotSpeeds.AUTO_SOURCE, 0.0, 0.0, 0.0))
                .toSpec();
    }

    /**
     * Creates a vision turn-assist command that writes angular feedback.
     *
     * @param speeds robot speed blender
     * @param targetVisible whether a target is available
     * @param yawErrorRadians target yaw error supplier
     * @param proportionalGain proportional turn gain
     * @param toleranceRadians completion tolerance
     * @return command spec
     */
    public static CommandSpec visionTurnAssist(
            RobotSpeeds speeds,
            BooleanSupplier targetVisible,
            DoubleSupplier yawErrorRadians,
            double proportionalGain,
            double toleranceRadians) {
        Objects.requireNonNull(speeds, "speeds");
        Objects.requireNonNull(targetVisible, "targetVisible");
        Objects.requireNonNull(yawErrorRadians, "yawErrorRadians");
        double gain = finiteOrZero(proportionalGain);
        double tolerance = Math.abs(finiteOrZero(toleranceRadians));
        BooleanSupplier aligned = () -> targetVisible.getAsBoolean()
                && Math.abs(finiteOrZero(yawErrorRadians.getAsDouble())) <= tolerance;
        return CommandSpec.create("visionTurnAssist")
                .onExecute(() -> {
                    double angular = targetVisible.getAsBoolean() ? finiteOrZero(yawErrorRadians.getAsDouble()) * gain : 0.0;
                    speeds.setSpeeds(RobotSpeeds.FEEDBACK_SOURCE, 0.0, 0.0, angular);
                })
                .until(aligned::getAsBoolean)
                .onEnd(() -> speeds.setSpeeds(RobotSpeeds.FEEDBACK_SOURCE, 0.0, 0.0, 0.0))
                .toSpec();
    }

    /**
     * Creates a field-relative waypoint follower command.
     *
     * @param speeds robot speed blender
     * @param currentPose current robot pose supplier
     * @param waypoints field-relative waypoint poses
     * @param cruiseMetersPerSecond maximum translation speed
     * @param angularGain heading proportional gain
     * @param toleranceMeters waypoint completion tolerance
     * @return command spec
     */
    public static CommandSpec followWaypoints(
            RobotSpeeds speeds,
            Supplier<PoseSnapshot> currentPose,
            List<PoseSnapshot> waypoints,
            double cruiseMetersPerSecond,
            double angularGain,
            double toleranceMeters) {
        Objects.requireNonNull(speeds, "speeds");
        Objects.requireNonNull(currentPose, "currentPose");
        List<PoseSnapshot> path = waypoints == null ? List.of() : List.copyOf(waypoints);
        double cruise = Math.abs(finiteOrZero(cruiseMetersPerSecond));
        double turnGain = finiteOrZero(angularGain);
        double tolerance = Math.max(1.0e-6, Math.abs(finiteOrZero(toleranceMeters)));
        int[] waypointIndex = new int[] {0};
        BooleanSupplier finished = () -> waypointIndex[0] >= path.size();
        return CommandSpec.create("followWaypoints")
                .onInitialize(() -> waypointIndex[0] = 0)
                .onExecute(() -> {
                    if (finished.getAsBoolean()) {
                        speeds.setSpeeds(RobotSpeeds.AUTO_SOURCE, 0.0, 0.0, 0.0);
                        return;
                    }
                    PoseSnapshot pose = currentPose.get();
                    if (pose == null) {
                        speeds.setSpeeds(RobotSpeeds.AUTO_SOURCE, 0.0, 0.0, 0.0);
                        return;
                    }
                    PoseSnapshot target = path.get(waypointIndex[0]);
                    double dx = finiteOrZero(target.xMeters() - pose.xMeters());
                    double dy = finiteOrZero(target.yMeters() - pose.yMeters());
                    double distance = Math.hypot(dx, dy);
                    if (distance <= tolerance) {
                        waypointIndex[0]++;
                        speeds.setFieldRelativeSpeeds(RobotSpeeds.AUTO_SOURCE, 0.0, 0.0, 0.0);
                        return;
                    }
                    double velocity = Math.min(cruise, distance);
                    double xVelocity = distance <= 1.0e-9 ? 0.0 : velocity * dx / distance;
                    double yVelocity = distance <= 1.0e-9 ? 0.0 : velocity * dy / distance;
                    double headingError = wrapRadians(target.headingRadians() - pose.headingRadians());
                    speeds.setFieldRelativeSpeeds(
                            RobotSpeeds.AUTO_SOURCE,
                            xVelocity,
                            yVelocity,
                            headingError * turnGain);
                })
                .until(finished::getAsBoolean)
                .onEnd(() -> speeds.setSpeeds(RobotSpeeds.AUTO_SOURCE, 0.0, 0.0, 0.0))
                .toSpec();
    }

    private static double finitePositive(double value, double fallback) {
        return Double.isFinite(value) && value > 0.0 ? value : fallback;
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    private static double wrapRadians(double value) {
        double wrapped = finiteOrZero(value);
        while (wrapped > Math.PI) {
            wrapped -= 2.0 * Math.PI;
        }
        while (wrapped < -Math.PI) {
            wrapped += 2.0 * Math.PI;
        }
        return wrapped;
    }
}
