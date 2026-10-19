package ca.frc6390.athena.auto;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
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

    /** One field-relative path pose. */
    public record Pose(Distance x, Distance y, Angle heading) {
        public Pose {
            Objects.requireNonNull(x, "x");
            Objects.requireNonNull(y, "y");
            Objects.requireNonNull(heading, "heading");
            if (!Double.isFinite(x.in(Meters)) || !Double.isFinite(y.in(Meters))
                    || !Double.isFinite(heading.in(Radians))) {
                throw new IllegalArgumentException("Path preview poses must be finite.");
            }
        }
    }

    /** Named path event and the field pose where its Action will be triggered. */
    public record Event(String name, Time time, Pose pose) {
        public Event {
            name = name == null || name.isBlank() ? "event" : name.trim();
            Objects.requireNonNull(time, "time");
            double seconds = time.in(Seconds);
            if (!Double.isFinite(seconds) || seconds < 0.0)
                throw new IllegalArgumentException("Event time must be finite and non-negative.");
            Objects.requireNonNull(pose, "pose");
        }
    }
}
