package ca.frc6390.athena.auto;

import ca.frc6390.athena.commands.CommandState;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

/**
 * Runtime marker graph for autonomous path events.
 */
public final class PathGraph {
    private final Map<String, CommandState> markers;
    private final Map<String, ActiveMarker> activeMarkers = new LinkedHashMap<>();

    private PathGraph(Map<String, CommandState> markers) {
        this.markers = Collections.unmodifiableMap(new LinkedHashMap<>(markers));
    }

    /**
     * Builds a path graph from autonomous routines.
     *
     * @param routines routines with marker bindings
     * @return path graph
     */
    public static PathGraph of(AutoRoutine... routines) {
        return of(routines == null ? List.of() : List.of(routines));
    }

    /**
     * Builds a path graph from autonomous routines.
     *
     * @param routines routines with marker bindings
     * @return path graph
     */
    public static PathGraph of(Collection<AutoRoutine> routines) {
        Map<String, CommandState> indexed = new LinkedHashMap<>();
        if (routines != null) {
            for (AutoRoutine routine : routines) {
                Objects.requireNonNull(routine, "routine");
                for (PathMarkerBinding binding : routine.markers()) {
                    CommandState previous = indexed.putIfAbsent(binding.marker(), binding.state());
                    if (previous != null) {
                        throw new IllegalArgumentException("Duplicate path marker '" + binding.marker() + "'.");
                    }
                }
            }
        }
        return new PathGraph(indexed);
    }

    /**
     * Returns marker names in registration order.
     *
     * @return marker names
     */
    public Collection<String> markerNames() {
        return markers.keySet();
    }

    /**
     * Finds a marker command by name.
     *
     * @param marker marker name
     * @return command state if present
     */
    public Optional<CommandState> marker(String marker) {
        return Optional.ofNullable(markers.get(normalize(marker)));
    }

    /**
     * Executes one cycle for a marker command.
     *
     * @param marker marker name
     * @return true when the marker command finished this cycle
     */
    public boolean trigger(String marker) {
        String key = normalize(marker);
        CommandState state = markers.get(key);
        if (state == null) {
            throw new IllegalArgumentException("Unknown path marker '" + key + "'.");
        }
        ActiveMarker active = activeMarkers.computeIfAbsent(key, ignored -> new ActiveMarker(state));
        boolean finished = active.execute();
        if (finished) {
            activeMarkers.remove(key);
        }
        return finished;
    }

    /**
     * Ends all active marker commands.
     *
     * @param interrupted true when commands are interrupted
     * @return this graph
     */
    public PathGraph endAll(boolean interrupted) {
        for (ActiveMarker marker : activeMarkers.values()) {
            marker.end(interrupted);
        }
        activeMarkers.clear();
        return this;
    }

    private static String normalize(String marker) {
        return marker == null || marker.isBlank() ? "marker" : marker.trim();
    }

    private static final class ActiveMarker {
        private final CommandState state;
        private boolean initialized;

        private ActiveMarker(CommandState state) {
            this.state = Objects.requireNonNull(state, "state");
        }

        private boolean execute() {
            if (!initialized) {
                state.onInitialize().run();
                initialized = true;
            }
            state.onExecute().run();
            if (state.isFinished().getAsBoolean()) {
                end(false);
                return true;
            }
            return false;
        }

        private void end(boolean interrupted) {
            if (initialized) {
                state.onEnd().run();
                initialized = false;
            }
        }
    }
}
