package ca.frc6390.athena.mechanism.core;

import java.util.Map;
import java.util.Objects;

/**
 * Introspected mechanism structure.
 *
 * @param name mechanism name
 * @param mechanism mechanism instance
 * @param children child mechanisms keyed by field name
 * @param states mechanism states keyed by field name
 * @param initialStateName initial state field name
 * @param initialState initial state
 * @param refs hardware and signal refs keyed by field name
 */
public record MechanismDefinition(
        String name,
        Mechanism mechanism,
        Map<String, Mechanism> children,
        Map<String, MechanismState> states,
        String initialStateName,
        MechanismState initialState,
        Map<String, Object> refs) {
    public MechanismDefinition {
        name = name == null || name.isBlank() ? "mechanism" : name;
        Objects.requireNonNull(mechanism, "mechanism");
        children = Map.copyOf(children);
        states = Map.copyOf(states);
        if (initialState == null && !states.isEmpty()) {
            Map.Entry<String, MechanismState> first = states.entrySet().iterator().next();
            initialStateName = first.getKey();
            initialState = first.getValue();
        }
        initialStateName = initialStateName == null ? "" : initialStateName;
        refs = Map.copyOf(refs);
    }
}
