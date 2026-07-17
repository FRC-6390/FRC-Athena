package ca.frc6390.athena.mechanism.core;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Collections;

/** The dashboard-independent, hierarchical telemetry contract for registered mechanisms. */
public final class TelemetrySchema {
    public enum Group {
        STATE("State"), VALUES("Values"), ACTIONS("Actions"), DEVICES("Devices"),
        CONTROLS("Controls"), HOOKS("Hooks"), TRACE("Trace");
        private final String path;
        Group(String path) { this.path = path; }
        public String path() { return path; }
    }

    private final TelemetryNode root;
    private final Map<String, TelemetryValue> values;
    private final Map<String, TelemetryNode> nodesByPath;

    TelemetrySchema(TelemetryNode root) {
        this.root = Objects.requireNonNull(root, "root");
        Map<String, TelemetryValue> flattenedValues = new LinkedHashMap<>();
        Map<String, TelemetryNode> flattenedNodes = new LinkedHashMap<>();
        flatten(root, "", flattenedValues, flattenedNodes);
        values = Collections.unmodifiableMap(flattenedValues);
        nodesByPath = Collections.unmodifiableMap(flattenedNodes);
    }
    public TelemetryNode root() { return root; }

    /** Finds a group using a slash-separated path. */
    public Optional<TelemetryNode> find(String path) {
        if (path == null || path.isBlank() || "/".equals(path)) return Optional.of(root);
        int start = 0;
        int end = path.length();
        while (start < end && path.charAt(start) == '/') start++;
        while (end > start && path.charAt(end - 1) == '/') end--;
        String key = start == 0 && end == path.length() ? path : path.substring(start, end);
        if (key.contains("//")) {
            StringBuilder normalized = new StringBuilder(key.length());
            boolean slash = false;
            for (int index = 0; index < key.length(); index++) {
                char character = key.charAt(index);
                if (character == '/') {
                    if (!slash) normalized.append(character);
                    slash = true;
                } else {
                    normalized.append(character);
                    slash = false;
                }
            }
            key = normalized.toString();
        }
        return Optional.ofNullable(nodesByPath.get(key));
    }

    /** Flattens value endpoints for transports that publish path-addressed topics. */
    public Map<String, TelemetryValue> values() {
        return values;
    }

    private static void flatten(
            TelemetryNode node,
            String path,
            Map<String, TelemetryValue> values,
            Map<String, TelemetryNode> nodes) {
        nodes.put(path, node);
        node.values().forEach((name, value) -> values.put(join(path, name), value));
        node.children().forEach((name, child) ->
                flatten(child, join(path, name), values, nodes));
    }

    private static String join(String left, String right) {
        return left.isEmpty() ? right : left + "/" + right;
    }
}
