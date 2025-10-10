package ca.frc6390.athena.mechanisms.superstructure;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Global registry that associates {@link Enum} superstructure states with their compiled
 * {@link SuperstructureStateDef}. The registry enables enum constants to satisfy Athena's
 * {@link ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider} contract without forcing teams
 * to thread a definition reference through every call-site.
 */
final class SuperstructureStateRegistry {

    private static final Map<Enum<?>, SuperstructureStateDef> DEFINITIONS = new ConcurrentHashMap<>();

    private SuperstructureStateRegistry() {}

    static void register(Enum<?> state, SuperstructureStateDef definition) {
        DEFINITIONS.put(state, definition);
    }

    static SuperstructureStateDef get(Enum<?> state) {
        SuperstructureStateDef definition = DEFINITIONS.get(state);
        if (definition == null) {
            throw new IllegalStateException(
                "Superstructure state " + state.name() + " has not been registered. "
                    + "Did you build a SuperstructureConfig before using it?");
        }
        return definition;
    }
}
