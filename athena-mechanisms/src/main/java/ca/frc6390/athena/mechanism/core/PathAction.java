package ca.frc6390.athena.mechanism.core;

import java.util.Objects;
import java.util.OptionalDouble;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Autonomous path request that can be composed anywhere a mechanism Action can be used.
 */
public record PathAction(
        String source,
        String name,
        int splitIndex,
        double expectedSeconds,
        boolean resetsOdometry,
        boolean onlyResetsPose,
        Map<String, Action> markers) implements Action {
    public PathAction {
        source = source == null || source.isBlank() ? "path" : source.trim();
        name = name == null || name.isBlank() ? "default" : name.trim();
        if (splitIndex < -1) throw new IllegalArgumentException("Path split index must be -1 or non-negative.");
        markers = markers == null ? Map.of() : Map.copyOf(new LinkedHashMap<>(markers));
    }

    public PathAction(String source, String name) {
        this(source, name, -1, Double.NaN, false, false, Map.of());
    }

    public PathAction(String source, String name, double expectedSeconds) {
        this(source, name, -1, expectedSeconds, false, false, Map.of());
    }

    public PathAction seconds(double seconds) {
        return copy(splitIndex, seconds, resetsOdometry, onlyResetsPose, markers);
    }

    /** Returns an Action for one zero-based split of this path. */
    public PathAction split(int index) {
        if (index < 0) throw new IllegalArgumentException("Path split index must be non-negative.");
        return copy(index, expectedSeconds, resetsOdometry, onlyResetsPose, markers);
    }

    /** Resets localization to the path's initial pose when this Action starts. */
    public PathAction resetOdometry() {
        return copy(splitIndex, expectedSeconds, true, false, markers);
    }

    /** Resets localization to the path's initial pose without following the path. */
    public PathAction resetPoseOnly() {
        return copy(splitIndex, expectedSeconds, true, true, Map.of());
    }

    /** Starts an ordinary Athena Action when the named path marker is crossed. */
    public PathAction marker(String marker, Action action) {
        String key = marker == null || marker.isBlank() ? "marker" : marker.trim();
        Map<String, Action> updated = new LinkedHashMap<>(markers);
        if (updated.putIfAbsent(key, Objects.requireNonNull(action, "action")) != null) {
            throw new IllegalArgumentException("Duplicate path marker '" + key + "'.");
        }
        return copy(splitIndex, expectedSeconds, resetsOdometry, onlyResetsPose, updated);
    }

    public OptionalDouble expectedDurationSeconds() {
        return Double.isFinite(expectedSeconds) ? OptionalDouble.of(expectedSeconds) : OptionalDouble.empty();
    }

    public String key() {
        return source + ":" + name + (splitIndex >= 0 ? ":split:" + splitIndex : "");
    }

    public boolean samePath(PathAction other) {
        return other != null
                && Objects.equals(source, other.source)
                && Objects.equals(name, other.name)
                && splitIndex == other.splitIndex;
    }

    private PathAction copy(
            int split,
            double seconds,
            boolean reset,
            boolean resetOnly,
            Map<String, Action> markerActions) {
        return new PathAction(source, name, split, seconds, reset, resetOnly, markerActions);
    }
}
