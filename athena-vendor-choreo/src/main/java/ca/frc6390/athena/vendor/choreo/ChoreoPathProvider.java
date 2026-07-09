package ca.frc6390.athena.vendor.choreo;

import java.util.Arrays;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;

import ca.frc6390.athena.auto.PathMarkerBinding;
import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.mechanism.core.PathAction;
import ca.frc6390.athena.mechanism.core.Paths;
import choreo.Choreo;
import choreo.trajectory.EventMarker;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;

/**
 * Choreo-backed provider for Athena path Actions and trajectories.
 */
public final class ChoreoPathProvider {
    /**
     * Source key used by Choreo path Actions.
     */
    public static final String KEY = "choreo";

    private final ChoreoClient client;
    private final Map<String, PathAction> pathCache = new ConcurrentHashMap<>();
    private final Map<String, Optional<Trajectory<? extends TrajectorySample<?>>>> trajectoryCache =
            new ConcurrentHashMap<>();
    private volatile List<String> pathNameCache;

    /**
     * Creates a provider that delegates to ChoreoLib's global trajectory loader.
     */
    public ChoreoPathProvider() {
        this(new ChoreoLibClient());
    }

    ChoreoPathProvider(ChoreoClient client) {
        this.client = Objects.requireNonNull(client, "client");
    }

    /**
     * Creates a Choreo path Action.
     *
     * @param pathName Choreo trajectory name
     * @return path Action
     */
    public PathAction path(String pathName) {
        return pathCache.computeIfAbsent(normalize(pathName), Paths::choreo);
    }

    /**
     * Loads a real Choreo trajectory.
     *
     * @param pathName Choreo trajectory name
     * @return trajectory if ChoreoLib can load it
     */
    public Optional<Trajectory<? extends TrajectorySample<?>>> trajectory(String pathName) {
        return trajectoryCache.computeIfAbsent(normalize(pathName), client::loadTrajectory);
    }

    /**
     * Returns trajectory names discovered by ChoreoLib.
     *
     * @return available trajectory names
     */
    public List<String> pathNames() {
        List<String> names = pathNameCache;
        if (names == null) {
            names = client.trajectoryNames();
            pathNameCache = names;
        }
        return names;
    }

    /**
     * Returns normalized marker names from a Choreo trajectory.
     *
     * @param pathName Choreo trajectory name
     * @return marker names in trajectory order, or an empty list when the trajectory is missing
     */
    public List<String> markerNames(String pathName) {
        return trajectory(pathName)
                .map(ChoreoPathProvider::markerNames)
                .orElseGet(List::of);
    }

    /**
     * Converts Choreo trajectory markers into Athena marker bindings.
     *
     * @param pathName Choreo trajectory name
     * @param commandsByMarker command Actions keyed by marker name
     * @return marker bindings in trajectory order, or an empty list when the trajectory is missing
     */
    public List<PathMarkerBinding> markerBindings(String pathName, Map<String, CommandAction> commandsByMarker) {
        Objects.requireNonNull(commandsByMarker, "commandsByMarker");
        Map<String, CommandAction> normalizedCommands = new LinkedHashMap<>();
        for (Map.Entry<String, CommandAction> entry : commandsByMarker.entrySet()) {
            String marker = normalizeMarker(entry.getKey());
            CommandAction command = Objects.requireNonNull(entry.getValue(), "marker command");
            CommandAction previous = normalizedCommands.putIfAbsent(marker, command);
            if (previous != null) {
                throw new IllegalArgumentException("Duplicate path marker command '" + marker + "'.");
            }
        }

        List<PathMarkerBinding> bindings = new ArrayList<>();
        for (String marker : markerNames(pathName)) {
            CommandAction command = normalizedCommands.get(marker);
            if (command == null) {
                throw new IllegalArgumentException("Missing command for Choreo marker '" + marker + "'.");
            }
            bindings.add(new PathMarkerBinding(marker, command));
        }
        return List.copyOf(bindings);
    }

    private static List<String> markerNames(Trajectory<? extends TrajectorySample<?>> trajectory) {
        LinkedHashSet<String> names = new LinkedHashSet<>();
        for (EventMarker marker : trajectory.events()) {
            names.add(normalizeMarker(marker.event));
        }
        return List.copyOf(names);
    }

    private static String normalize(String pathName) {
        return pathName == null || pathName.isBlank() ? "default" : pathName.trim();
    }

    private static String normalizeMarker(String marker) {
        return marker == null || marker.isBlank() ? "marker" : marker.trim();
    }

    interface ChoreoClient {
        Optional<Trajectory<? extends TrajectorySample<?>>> loadTrajectory(String trajectoryName);

        List<String> trajectoryNames();
    }

    private static final class ChoreoLibClient implements ChoreoClient {
        @Override
        public Optional<Trajectory<? extends TrajectorySample<?>>> loadTrajectory(String trajectoryName) {
            return loadAnyTrajectory(trajectoryName);
        }

        @Override
        public List<String> trajectoryNames() {
            return List.copyOf(Arrays.asList(Choreo.availableTrajectories()));
        }

        @SuppressWarnings({"rawtypes", "unchecked"})
        private static Optional<Trajectory<? extends TrajectorySample<?>>> loadAnyTrajectory(String trajectoryName) {
            Optional<? extends Trajectory> trajectory = Choreo.loadTrajectory(trajectoryName);
            return (Optional<Trajectory<? extends TrajectorySample<?>>>) (Optional<?>) trajectory;
        }
    }
}
