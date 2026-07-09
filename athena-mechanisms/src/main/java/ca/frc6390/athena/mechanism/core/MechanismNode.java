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
        Map<String, State> states,
        String initialStateName,
        State initialState,
        Map<String, Object> declarations,
        Map<String, HookBinding> hooks) {
    public MechanismNode(
            String name,
            Mechanism mechanism,
            Map<String, Mechanism> children,
            Map<String, State> states,
            Map<String, Object> declarations,
            Map<String, HookBinding> hooks) {
        this(name, mechanism, children, states, firstStateName(states), firstState(states), declarations, hooks);
    }

    public MechanismNode {
        name = name == null || name.isBlank() ? "mechanism" : name;
        Objects.requireNonNull(mechanism, "mechanism");
        children = Collections.unmodifiableMap(new LinkedHashMap<>(children));
        states = Collections.unmodifiableMap(new LinkedHashMap<>(states));
        initialStateName = initialStateName == null ? "" : initialStateName;
        declarations = Collections.unmodifiableMap(new LinkedHashMap<>(declarations));
        hooks = Collections.unmodifiableMap(new LinkedHashMap<>(hooks));
    }

    private static String firstStateName(Map<String, State> states) {
        return states.isEmpty() ? "" : states.keySet().iterator().next();
    }

    private static State firstState(Map<String, State> states) {
        return states.isEmpty() ? null : states.values().iterator().next();
    }
}
