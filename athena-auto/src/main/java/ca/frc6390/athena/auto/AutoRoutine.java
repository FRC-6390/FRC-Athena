package ca.frc6390.athena.auto;

import ca.frc6390.athena.commands.CommandAction;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * Named autonomous routine that creates the command Action selected by auto runtime.
 *
 * @param name routine name
 * @param stateFactory factory for a fresh command Action
 * @param markers marker command bindings keyed by marker name
 */
public record AutoRoutine(String name, Supplier<CommandAction> stateFactory, List<PathMarkerBinding> markers) {
    public AutoRoutine {
        name = name == null || name.isBlank() ? "auto" : name.trim();
        Objects.requireNonNull(stateFactory, "stateFactory");
        markers = validateMarkers(markers);
    }

    /**
     * Creates an autonomous routine without marker bindings.
     *
     * @param name routine name
     * @param stateFactory factory for a fresh command Action
     */
    public AutoRoutine(String name, Supplier<CommandAction> stateFactory) {
        this(name, stateFactory, List.of());
    }

    /**
     * Creates the routine Action.
     *
     * @return command Action
     */
    public CommandAction Action() {
        return Objects.requireNonNull(stateFactory.get(), "stateFactory returned null");
    }

    private static List<PathMarkerBinding> validateMarkers(Collection<PathMarkerBinding> bindings) {
        if (bindings == null || bindings.isEmpty()) {
            return List.of();
        }
        Map<String, PathMarkerBinding> indexed = new LinkedHashMap<>();
        for (PathMarkerBinding binding : bindings) {
            Objects.requireNonNull(binding, "marker binding");
            PathMarkerBinding previous = indexed.putIfAbsent(binding.marker(), binding);
            if (previous != null) {
                throw new IllegalArgumentException("Duplicate path marker '" + binding.marker() + "'.");
            }
        }
        return List.copyOf(indexed.values());
    }
}
