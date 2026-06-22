package ca.frc6390.athena.telemetry.networktables;

import java.util.Arrays;
import java.util.stream.Collectors;

import ca.frc6390.athena.telemetry.TelemetryKey;

/**
 * Normalizes Athena telemetry keys into NetworkTables-style paths.
 */
public final class NetworkTablePath {
    private static final String DEFAULT_ROOT = "/Athena";

    private NetworkTablePath() {
    }

    /**
     * Converts a telemetry key to a NetworkTables path under `/Athena`.
     *
     * @param key telemetry key
     * @return normalized path
     */
    public static String from(TelemetryKey key) {
        return from(DEFAULT_ROOT, key);
    }

    /**
     * Converts a telemetry key to a NetworkTables path under a root path.
     *
     * @param root root path
     * @param key telemetry key
     * @return normalized path
     */
    public static String from(String root, TelemetryKey key) {
        return normalize(root) + "/" + normalize(key.path()).substring(1);
    }

    /**
     * Normalizes slashes and trims blank segments.
     *
     * @param path input path
     * @return normalized absolute path
     */
    public static String normalize(String path) {
        String raw = path == null || path.isBlank() ? DEFAULT_ROOT : path;
        String body = Arrays.stream(raw.split("/+"))
                .filter(segment -> !segment.isBlank())
                .collect(Collectors.joining("/"));
        return "/" + body;
    }
}
