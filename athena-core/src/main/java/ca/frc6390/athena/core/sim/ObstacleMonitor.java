package ca.frc6390.athena.core.sim;

import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;

/**
 * Checks planned paths against obstacle fields and signals when a replan is needed.
 */
public final class ObstacleMonitor {
    private final ObstacleField obstacleField;
    private Supplier<Pose2d> robotPoseSupplier;
    private Supplier<List<Pose2d>> plannedPathSupplier;
    private double robotRadiusMeters = 0.4;
    private double lookaheadSeconds = 2.0;
    private double assumedPathSpeedMetersPerSecond = 1.0;
    private double replanHoldSeconds = 0.25;
    private double replanActiveUntilSeconds = Double.NEGATIVE_INFINITY;
    private Pose2d lastCollisionPose;

    public ObstacleMonitor(
            ObstacleField obstacleField,
            Supplier<Pose2d> robotPoseSupplier,
            Supplier<List<Pose2d>> plannedPathSupplier) {
        this.obstacleField = Objects.requireNonNull(obstacleField, "obstacleField");
        this.robotPoseSupplier = Objects.requireNonNull(robotPoseSupplier, "robotPoseSupplier");
        this.plannedPathSupplier = Objects.requireNonNull(plannedPathSupplier, "plannedPathSupplier");
    }

    public boolean update() {
        return update(Timer.getFPGATimestamp());
    }

    public boolean update(double nowSeconds) {
        boolean collision = predictCollision();
        if (collision) {
            replanActiveUntilSeconds = nowSeconds + replanHoldSeconds;
        }
        return nowSeconds <= replanActiveUntilSeconds;
    }

    public boolean shouldReplan() {
        return Timer.getFPGATimestamp() <= replanActiveUntilSeconds;
    }

    public Pose2d getLastCollisionPose() {
        return lastCollisionPose;
    }

    public ObstacleMonitor setRobotRadiusMeters(double robotRadiusMeters) {
        if (Double.isFinite(robotRadiusMeters) && robotRadiusMeters >= 0.0) {
            this.robotRadiusMeters = robotRadiusMeters;
        }
        return this;
    }

    public ObstacleMonitor setLookaheadSeconds(double lookaheadSeconds) {
        if (Double.isFinite(lookaheadSeconds) && lookaheadSeconds > 0.0) {
            this.lookaheadSeconds = lookaheadSeconds;
        }
        return this;
    }

    public ObstacleMonitor setAssumedPathSpeedMetersPerSecond(double speed) {
        if (Double.isFinite(speed) && speed > 0.0) {
            this.assumedPathSpeedMetersPerSecond = speed;
        }
        return this;
    }

    public ObstacleMonitor setReplanHoldSeconds(double holdSeconds) {
        if (Double.isFinite(holdSeconds) && holdSeconds >= 0.0) {
            this.replanHoldSeconds = holdSeconds;
        }
        return this;
    }

    public ObstacleMonitor setRobotPoseSupplier(Supplier<Pose2d> supplier) {
        this.robotPoseSupplier = Objects.requireNonNull(supplier, "supplier");
        return this;
    }

    public ObstacleMonitor setPlannedPathSupplier(Supplier<List<Pose2d>> supplier) {
        this.plannedPathSupplier = Objects.requireNonNull(supplier, "supplier");
        return this;
    }

    private boolean predictCollision() {
        List<Pose2d> path = plannedPathSupplier.get();
        Pose2d robotPose = robotPoseSupplier.get();
        if (path == null || path.isEmpty() || robotPose == null) {
            return false;
        }
        double horizonDistance = lookaheadSeconds * assumedPathSpeedMetersPerSecond;
        if (!Double.isFinite(horizonDistance) || horizonDistance <= 0.0) {
            horizonDistance = 2.0;
        }
        int startIndex = findClosestIndex(path, robotPose.getTranslation());
        double traveled = 0.0;
        Translation2d last = path.get(startIndex).getTranslation();
        for (int i = startIndex; i < path.size(); i++) {
            Pose2d pose = path.get(i);
            Translation2d current = pose.getTranslation();
            traveled += current.getDistance(last);
            last = current;
            if (traveled > horizonDistance) {
                break;
            }
            if (obstacleField.isCollision(current, robotRadiusMeters)) {
                lastCollisionPose = pose;
                return true;
            }
        }
        return false;
    }

    private static int findClosestIndex(List<Pose2d> path, Translation2d robot) {
        int best = 0;
        double bestDist = Double.POSITIVE_INFINITY;
        for (int i = 0; i < path.size(); i++) {
            Translation2d point = path.get(i).getTranslation();
            double dist = point.getDistance(robot);
            if (dist < bestDist) {
                bestDist = dist;
                best = i;
            }
        }
        return best;
    }
}
