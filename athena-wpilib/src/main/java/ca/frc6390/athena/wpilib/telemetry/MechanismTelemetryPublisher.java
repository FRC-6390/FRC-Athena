package ca.frc6390.athena.wpilib.telemetry;

import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.runtime.geometry.Circle2d;
import ca.frc6390.athena.runtime.geometry.Geometry2d;
import ca.frc6390.athena.runtime.geometry.Point2d;
import ca.frc6390.athena.runtime.geometry.Polygon2d;
import ca.frc6390.athena.runtime.geometry.Rectangle2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.HashSet;
import java.util.Set;
import edu.wpi.first.wpilibj.DriverStation;

/** Publishes sparse custom telemetry and synchronizes live tunables over NT4. */
public final class MechanismTelemetryPublisher {
    private static final double CUSTOM_PERIOD_SECONDS = 0.10;
    private final NetworkTableInstance instance;
    private final Map<String, NetworkTableEntry> entries = new LinkedHashMap<>();
    private final Map<String, StructArrayPublisher<Pose2d>> geometryPublishers = new LinkedHashMap<>();
    private final Set<String> reportedFailures = new HashSet<>();
    private double lastCustomPublish = Double.NEGATIVE_INFINITY;

    public MechanismTelemetryPublisher() { this(NetworkTableInstance.getDefault()); }

    MechanismTelemetryPublisher(NetworkTableInstance instance) {
        this.instance = Objects.requireNonNull(instance, "instance");
    }

    public void publish(Map<String, TelemetryValue> values) {
        publish(values, Timer.getFPGATimestamp());
    }

    void publish(Map<String, TelemetryValue> values, double timestampSeconds) {
        Objects.requireNonNull(values, "values");
        boolean publishCustom = timestampSeconds - lastCustomPublish >= CUSTOM_PERIOD_SECONDS;
        if (publishCustom) lastCustomPublish = timestampSeconds;
        values.forEach((path, value) -> {
            if (!value.writable() && !publishCustom) return;
            String topic = "/Athena/Mechanisms/" + clean(path);
            try {
                if (value.type() == TelemetryValue.Type.GEOMETRY) {
                    publishGeometry(topic, (Geometry2d) value.value());
                    return;
                }
                NetworkTableEntry entry = entries.computeIfAbsent(topic, instance::getEntry);
                if (value.writable()) sync(entry, value); else write(entry, value);
            } catch (RuntimeException exception) {
                if (reportedFailures.add(path)) {
                    DriverStation.reportWarning(
                            "Athena telemetry '" + path + "' is unavailable: " + exception.getMessage(), false);
                }
            }
        });
    }

    private static void sync(NetworkTableEntry entry, TelemetryValue value) {
        Object current = value.value();
        if (!entry.exists()) {
            setDefault(entry, value.type(), current);
            return;
        }
        switch (value.type()) {
            case NUMBER -> value.set(entry.getDouble(((Number) current).doubleValue()));
            case BOOLEAN -> value.set(entry.getBoolean((Boolean) current));
            case STRING -> value.set(entry.getString(String.valueOf(current)));
            case GEOMETRY -> throw new IllegalArgumentException("Geometry telemetry is read-only.");
        }
    }

    private static void write(NetworkTableEntry entry, TelemetryValue value) {
        Object current = value.value();
        switch (value.type()) {
            case NUMBER -> entry.setDouble(((Number) current).doubleValue());
            case BOOLEAN -> entry.setBoolean((Boolean) current);
            case STRING -> entry.setString(String.valueOf(current));
            case GEOMETRY -> throw new IllegalArgumentException("Geometry requires a struct-array topic.");
        }
    }

    private static void setDefault(NetworkTableEntry entry, TelemetryValue.Type type, Object value) {
        switch (type) {
            case NUMBER -> entry.setDefaultDouble(((Number) value).doubleValue());
            case BOOLEAN -> entry.setDefaultBoolean((Boolean) value);
            case STRING -> entry.setDefaultString(String.valueOf(value));
            case GEOMETRY -> throw new IllegalArgumentException("Geometry telemetry is read-only.");
        }
    }

    private void publishGeometry(String topic, Geometry2d geometry) {
        geometryPublishers.computeIfAbsent(
                topic,
                key -> instance.getStructArrayTopic(key, Pose2d.struct).publish())
                .set(outline(geometry));
    }

    private static Pose2d[] outline(Geometry2d geometry) {
        if (geometry instanceof Rectangle2d rectangle) {
            return poses(
                    new Point2d(rectangle.minimumX(), rectangle.minimumY()),
                    new Point2d(rectangle.maximumX(), rectangle.minimumY()),
                    new Point2d(rectangle.maximumX(), rectangle.maximumY()),
                    new Point2d(rectangle.minimumX(), rectangle.maximumY()),
                    new Point2d(rectangle.minimumX(), rectangle.minimumY()));
        }
        if (geometry instanceof Polygon2d polygon) {
            Point2d[] points = new Point2d[polygon.vertices().size() + 1];
            for (int index = 0; index < polygon.vertices().size(); index++) {
                points[index] = polygon.vertices().get(index);
            }
            points[points.length - 1] = polygon.vertices().get(0);
            return poses(points);
        }
        if (geometry instanceof Circle2d circle) {
            Point2d[] points = new Point2d[33];
            for (int index = 0; index < points.length; index++) {
                double angle = 2.0 * Math.PI * index / (points.length - 1);
                points[index] = new Point2d(
                        circle.center().x() + circle.radius() * Math.cos(angle),
                        circle.center().y() + circle.radius() * Math.sin(angle));
            }
            return poses(points);
        }
        throw new IllegalArgumentException(
                "Only Rectangle2d, Polygon2d, and Circle2d can be visualized.");
    }

    private static Pose2d[] poses(Point2d... points) {
        Pose2d[] poses = new Pose2d[points.length];
        for (int index = 0; index < points.length; index++) {
            poses[index] = new Pose2d(points[index].x(), points[index].y(), Rotation2d.kZero);
        }
        return poses;
    }

    private static String clean(String path) {
        return path == null ? "value" : path.replaceAll("[^A-Za-z0-9_./-]", "_").replaceAll("^/+", "");
    }
}
