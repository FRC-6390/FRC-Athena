package ca.frc6390.athena.auto;

import java.util.List;
import java.util.Objects;

/** Vendor-neutral geometry for displaying one autonomous path before it runs. */
public record PathPreview(String key, List<Pose> poses, List<Event> events) {
    public PathPreview {
        key = key == null || key.isBlank() ? "path" : key.trim();
        poses = List.copyOf(Objects.requireNonNull(poses, "poses"));
        events = List.copyOf(Objects.requireNonNull(events, "events"));
    }

    public PathPreview(String key, List<Pose> poses) { this(key, poses, List.of()); }

    /** One field-relative path pose in meters and radians. */
    public record Pose(double xMeters, double yMeters, double headingRadians) {
        public Pose {
            if (!Double.isFinite(xMeters) || !Double.isFinite(yMeters) || !Double.isFinite(headingRadians)) {
                throw new IllegalArgumentException("Path preview poses must be finite.");
            }
        }
    }

    /** Named path event and the field pose where its Action will be triggered. */
    public record Event(String name, double timeSeconds, Pose pose) {
        public Event {
            name = name == null || name.isBlank() ? "event" : name.trim();
            if (!Double.isFinite(timeSeconds) || timeSeconds < 0.0)
                throw new IllegalArgumentException("Event time must be finite and non-negative.");
            Objects.requireNonNull(pose, "pose");
        }
    }
}
