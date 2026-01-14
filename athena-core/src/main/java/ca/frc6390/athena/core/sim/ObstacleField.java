package ca.frc6390.athena.core.sim;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Simple obstacle container for simulation and planner prototyping.
 */
public final class ObstacleField {

    public static final class CostFieldConfig {
        private double falloffMeters = 1.0;
        private double maxCost = 1.0;
        private double minDistanceMeters = 0.05;

        public double getFalloffMeters() {
            return falloffMeters;
        }

        public CostFieldConfig setFalloffMeters(double falloffMeters) {
            if (Double.isFinite(falloffMeters) && falloffMeters > 0.0) {
                this.falloffMeters = falloffMeters;
            }
            return this;
        }

        public double getMaxCost() {
            return maxCost;
        }

        public CostFieldConfig setMaxCost(double maxCost) {
            if (Double.isFinite(maxCost) && maxCost > 0.0) {
                this.maxCost = maxCost;
            }
            return this;
        }

        public double getMinDistanceMeters() {
            return minDistanceMeters;
        }

        public CostFieldConfig setMinDistanceMeters(double minDistanceMeters) {
            if (Double.isFinite(minDistanceMeters) && minDistanceMeters > 0.0) {
                this.minDistanceMeters = minDistanceMeters;
            }
            return this;
        }
    }

    private interface Obstacle {
        double distanceTo(Translation2d point);

        default double costAt(Translation2d point, CostFieldConfig config) {
            double distance = distanceTo(point);
            if (!Double.isFinite(distance)) {
                return 0.0;
            }
            if (distance <= 0.0) {
                return config.getMaxCost();
            }
            double scaled = Math.max(distance, config.getMinDistanceMeters());
            return config.getMaxCost() * Math.exp(-scaled / config.getFalloffMeters());
        }
    }

    private static final class PolygonObstacle implements Obstacle {
        private final String id;
        private final List<Translation2d> vertices;

        private PolygonObstacle(String id, List<Translation2d> vertices) {
            this.id = id;
            this.vertices = vertices;
        }

        @Override
        public double distanceTo(Translation2d point) {
            if (point == null || vertices.size() < 3) {
                return Double.POSITIVE_INFINITY;
            }
            if (pointInPolygon(point, vertices)) {
                return 0.0;
            }
            double min = Double.POSITIVE_INFINITY;
            for (int i = 0; i < vertices.size(); i++) {
                Translation2d a = vertices.get(i);
                Translation2d b = vertices.get((i + 1) % vertices.size());
                double dist = distanceToSegment(point, a, b);
                if (dist < min) {
                    min = dist;
                }
            }
            return min;
        }
    }

    private static final class CircleObstacle implements Obstacle {
        private final String id;
        private final Supplier<Translation2d> centerSupplier;
        private final double radiusMeters;

        private CircleObstacle(String id, Supplier<Translation2d> centerSupplier, double radiusMeters) {
            this.id = id;
            this.centerSupplier = centerSupplier;
            this.radiusMeters = radiusMeters;
        }

        @Override
        public double distanceTo(Translation2d point) {
            if (point == null) {
                return Double.POSITIVE_INFINITY;
            }
            Translation2d center = centerSupplier.get();
            if (center == null) {
                return Double.POSITIVE_INFINITY;
            }
            double dist = point.getDistance(center);
            return Math.max(0.0, dist - radiusMeters);
        }
    }

    private final List<Obstacle> obstacles = new ArrayList<>();
    private final CostFieldConfig costFieldConfig = new CostFieldConfig();

    public ObstacleField addStaticPolygon(String id, List<Translation2d> vertices) {
        Objects.requireNonNull(vertices, "vertices");
        List<Translation2d> copy = new ArrayList<>(vertices);
        if (copy.size() < 3) {
            return this;
        }
        obstacles.add(new PolygonObstacle(id, Collections.unmodifiableList(copy)));
        return this;
    }

    public ObstacleField addDynamicCircleFromPose(String id, Supplier<Pose2d> poseSupplier, double radiusMeters) {
        Objects.requireNonNull(poseSupplier, "poseSupplier");
        return addDynamicCircle(id, () -> {
            Pose2d pose = poseSupplier.get();
            return pose != null ? pose.getTranslation() : null;
        }, radiusMeters);
    }

    public ObstacleField addDynamicCircle(String id, Supplier<Translation2d> centerSupplier, double radiusMeters) {
        Objects.requireNonNull(centerSupplier, "centerSupplier");
        if (!Double.isFinite(radiusMeters) || radiusMeters <= 0.0) {
            return this;
        }
        obstacles.add(new CircleObstacle(id, centerSupplier, radiusMeters));
        return this;
    }

    public ObstacleField clear() {
        obstacles.clear();
        return this;
    }

    public CostFieldConfig getCostFieldConfig() {
        return costFieldConfig;
    }

    public boolean isCollision(Translation2d point, double radiusMeters) {
        if (point == null) {
            return false;
        }
        double radius = Double.isFinite(radiusMeters) ? Math.max(0.0, radiusMeters) : 0.0;
        for (Obstacle obstacle : obstacles) {
            if (obstacle.distanceTo(point) <= radius) {
                return true;
            }
        }
        return false;
    }

    public double costAt(Translation2d point) {
        if (point == null) {
            return 0.0;
        }
        double sum = 0.0;
        for (Obstacle obstacle : obstacles) {
            sum += obstacle.costAt(point, costFieldConfig);
        }
        return sum;
    }

    public double distanceToNearest(Translation2d point) {
        if (point == null) {
            return Double.POSITIVE_INFINITY;
        }
        double min = Double.POSITIVE_INFINITY;
        for (Obstacle obstacle : obstacles) {
            min = Math.min(min, obstacle.distanceTo(point));
        }
        return min;
    }

    private static boolean pointInPolygon(Translation2d point, List<Translation2d> vertices) {
        boolean inside = false;
        double x = point.getX();
        double y = point.getY();
        for (int i = 0, j = vertices.size() - 1; i < vertices.size(); j = i++) {
            double xi = vertices.get(i).getX();
            double yi = vertices.get(i).getY();
            double xj = vertices.get(j).getX();
            double yj = vertices.get(j).getY();
            boolean intersect = ((yi > y) != (yj > y))
                    && (x < (xj - xi) * (y - yi) / (yj - yi + 1e-9) + xi);
            if (intersect) {
                inside = !inside;
            }
        }
        return inside;
    }

    private static double distanceToSegment(Translation2d point, Translation2d a, Translation2d b) {
        double ax = a.getX();
        double ay = a.getY();
        double bx = b.getX();
        double by = b.getY();
        double px = point.getX();
        double py = point.getY();
        double dx = bx - ax;
        double dy = by - ay;
        double denom = dx * dx + dy * dy;
        if (denom <= 1e-9) {
            return Math.hypot(px - ax, py - ay);
        }
        double t = ((px - ax) * dx + (py - ay) * dy) / denom;
        t = Math.max(0.0, Math.min(1.0, t));
        double cx = ax + t * dx;
        double cy = ay + t * dy;
        return Math.hypot(px - cx, py - cy);
    }
}
