package ca.frc6390.athena.mechanisms.superstructure;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

/**
 * Shared context passed to each macro state while building the superstructure definition and reused
 * at runtime. Teams can subclass this to expose additional helpers (vision, LEDs, logging, etc.)
 * while still leveraging the convenience methods for registered mechanisms.
 */
public class SuperstructureContext {

    private final Map<String, SuperstructureMechanismBinding<?>> mechanisms = new HashMap<>();

    final void registerMechanism(String key, SuperstructureMechanismBinding<?> binding) {
        mechanisms.put(key, Objects.requireNonNull(binding));
    }

    @SuppressWarnings("unchecked")
    public <E extends Enum<E> & SetpointProvider<?>> SuperstructureMechanismBinding<E> mechanism(String key) {
        SuperstructureMechanismBinding<?> binding = mechanisms.get(key);
        if (binding == null) {
            throw new IllegalArgumentException("No mechanism registered with key " + key);
        }
        return (SuperstructureMechanismBinding<E>) binding;
    }

    final Map<String, SuperstructureMechanismBinding<?>> mechanismMapView() {
        return Map.copyOf(mechanisms);
    }
}
