package ca.frc6390.athena.mechanism.core;

import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

/**
 * Internal graph node for an introspected mechanism.
 */
public record MechanismNode(
        String name,
        Mechanism mechanism,
        Map<String, Mechanism> children,
        Map<String, Action> Actions,
        Map<String, Object> declarations,
        Map<String, HookBinding> hooks) {
    public MechanismNode {
        name = name == null || name.isBlank() ? "mechanism" : name;
        Objects.requireNonNull(mechanism, "mechanism");
        children = Collections.unmodifiableMap(new LinkedHashMap<>(children));
        Actions = Collections.unmodifiableMap(new LinkedHashMap<>(Actions));
        declarations = Collections.unmodifiableMap(new LinkedHashMap<>(declarations));
        hooks = Collections.unmodifiableMap(new LinkedHashMap<>(hooks));
    }
}
