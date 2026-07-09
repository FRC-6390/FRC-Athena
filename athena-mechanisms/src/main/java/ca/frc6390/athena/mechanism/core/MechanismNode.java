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
        String initialStateName,
        Action initialState,
        Map<String, Object> declarations,
        Map<String, HookBinding> hooks) {
    public MechanismNode(
            String name,
            Mechanism mechanism,
            Map<String, Mechanism> children,
            Map<String, Action> Actions,
            Map<String, Object> declarations,
            Map<String, HookBinding> hooks) {
        this(name, mechanism, children, Actions, firstStateName(Actions), firstState(Actions), declarations, hooks);
    }

    public MechanismNode {
        name = name == null || name.isBlank() ? "mechanism" : name;
        Objects.requireNonNull(mechanism, "mechanism");
        children = Collections.unmodifiableMap(new LinkedHashMap<>(children));
        Actions = Collections.unmodifiableMap(new LinkedHashMap<>(Actions));
        initialStateName = initialStateName == null ? "" : initialStateName;
        declarations = Collections.unmodifiableMap(new LinkedHashMap<>(declarations));
        hooks = Collections.unmodifiableMap(new LinkedHashMap<>(hooks));
    }

    private static String firstStateName(Map<String, Action> Actions) {
        return Actions.isEmpty() ? "" : Actions.keySet().iterator().next();
    }

    private static Action firstState(Map<String, Action> Actions) {
        return Actions.isEmpty() ? null : Actions.values().iterator().next();
    }
}
