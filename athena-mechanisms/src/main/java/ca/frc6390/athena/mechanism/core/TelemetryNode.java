package ca.frc6390.athena.mechanism.core;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

/** An immutable group in Athena's mechanism-scoped telemetry tree. */
public final class TelemetryNode {
    public enum Kind { ROOT, MECHANISM, GROUP }

    private final String name;
    private final Kind kind;
    private final Map<String, TelemetryNode> children;
    private final Map<String, TelemetryValue> values;
    private final Map<String, TelemetryAction> actions;

    TelemetryNode(String name, Kind kind, Map<String, TelemetryNode> children,
            Map<String, TelemetryValue> values, Map<String, TelemetryAction> actions) {
        this.name = Objects.requireNonNull(name, "name");
        this.kind = Objects.requireNonNull(kind, "kind");
        this.children = Collections.unmodifiableMap(new LinkedHashMap<>(children));
        this.values = Collections.unmodifiableMap(new LinkedHashMap<>(values));
        this.actions = Collections.unmodifiableMap(new LinkedHashMap<>(actions));
    }

    public String name() { return name; }
    public Kind kind() { return kind; }
    public Map<String, TelemetryNode> children() { return children; }
    public Map<String, TelemetryValue> values() { return values; }
    public Map<String, TelemetryAction> actions() { return actions; }
    public Optional<TelemetryNode> child(String name) { return Optional.ofNullable(children.get(name)); }
}
