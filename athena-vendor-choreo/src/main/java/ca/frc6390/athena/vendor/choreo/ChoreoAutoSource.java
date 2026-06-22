package ca.frc6390.athena.vendor.choreo;

import ca.frc6390.athena.auto.AutoSource;
import ca.frc6390.athena.commands.CommandSpec;
import choreo.Choreo;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;
import java.util.Optional;

/**
 * Choreo autonomous source backed by ChoreoLib trajectory loading.
 */
public final class ChoreoAutoSource implements AutoSource {
    /**
     * Source key used in Athena auto declarations.
     */
    public static final String KEY = "choreo";

    private final ChoreoClient client;

    /**
     * Creates a source that delegates to ChoreoLib's global trajectory loader.
     */
    public ChoreoAutoSource() {
        this(new ChoreoLibClient());
    }

    ChoreoAutoSource(ChoreoClient client) {
        this.client = Objects.requireNonNull(client, "client");
    }

    @Override
    public CommandSpec load(String routinePath) {
        return CommandSpec.create(KEY + ":" + normalize(routinePath)).toSpec();
    }

    /**
     * Loads a real Choreo trajectory.
     *
     * @param trajectoryName Choreo trajectory name
     * @return trajectory if ChoreoLib can load it
     */
    public Optional<Trajectory<? extends TrajectorySample<?>>> loadTrajectory(String trajectoryName) {
        return client.loadTrajectory(normalize(trajectoryName));
    }

    /**
     * Returns trajectory names discovered by ChoreoLib.
     *
     * @return available trajectory names
     */
    public List<String> trajectoryNames() {
        return client.trajectoryNames();
    }

    private static String normalize(String routinePath) {
        return routinePath == null || routinePath.isBlank() ? "default" : routinePath.trim();
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
