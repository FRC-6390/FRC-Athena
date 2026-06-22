package ca.frc6390.athena.vendor.choreo;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Objects;

/**
 * Thin Athena-facing wrapper around Choreo's {@link AutoFactory} command API.
 */
public final class ChoreoAutoFactoryAdapter {
    private final FactoryClient client;

    /**
     * Creates an adapter backed by a real Choreo {@link AutoFactory}.
     *
     * @param factory configured Choreo auto factory
     */
    public ChoreoAutoFactoryAdapter(AutoFactory factory) {
        this(new ChoreoFactoryClient(factory));
    }

    /**
     * Creates an adapter backed by a custom client.
     *
     * @param client command-loading client
     */
    public ChoreoAutoFactoryAdapter(FactoryClient client) {
        this.client = Objects.requireNonNull(client, "client");
    }

    /**
     * Creates a command that runs a Choreo trajectory.
     *
     * @param trajectoryName Choreo trajectory name
     * @return WPILib command from the backing factory
     */
    public Command trajectoryCommand(String trajectoryName) {
        return client.trajectoryCommand(normalize(trajectoryName));
    }

    /**
     * Creates a command that runs one split of a Choreo trajectory.
     *
     * @param trajectoryName Choreo trajectory name
     * @param splitIndex split index to run
     * @return WPILib command from the backing factory
     */
    public Command trajectoryCommand(String trajectoryName, int splitIndex) {
        return client.trajectoryCommand(normalize(trajectoryName), splitIndex);
    }

    /**
     * Creates a command that resets odometry from a Choreo trajectory.
     *
     * @param trajectoryName Choreo trajectory name
     * @return WPILib command from the backing factory
     */
    public Command resetOdometryCommand(String trajectoryName) {
        return client.resetOdometryCommand(normalize(trajectoryName));
    }

    /**
     * Creates a command that resets odometry from one split of a Choreo trajectory.
     *
     * @param trajectoryName Choreo trajectory name
     * @param splitIndex split index to read
     * @return WPILib command from the backing factory
     */
    public Command resetOdometryCommand(String trajectoryName, int splitIndex) {
        return client.resetOdometryCommand(normalize(trajectoryName), splitIndex);
    }

    /**
     * Creates a command for a named Choreo routine.
     *
     * @param routineName Choreo routine name
     * @return WPILib command from the backing factory
     */
    public Command routineCommand(String routineName) {
        return client.routineCommand(normalize(routineName));
    }

    /**
     * Creates a command that warms Choreo's trajectory cache.
     *
     * @return WPILib command from the backing factory
     */
    public Command warmupCommand() {
        return client.warmupCommand();
    }

    private static String normalize(String name) {
        return name == null || name.isBlank() ? "default" : name.trim();
    }

    /**
     * Minimal command-producing surface used by the adapter.
     */
    public interface FactoryClient {
        /**
         * Creates a trajectory command.
         *
         * @param trajectoryName Choreo trajectory name
         * @return WPILib command
         */
        Command trajectoryCommand(String trajectoryName);

        /**
         * Creates a split trajectory command.
         *
         * @param trajectoryName Choreo trajectory name
         * @param splitIndex split index
         * @return WPILib command
         */
        Command trajectoryCommand(String trajectoryName, int splitIndex);

        /**
         * Creates an odometry reset command.
         *
         * @param trajectoryName Choreo trajectory name
         * @return WPILib command
         */
        Command resetOdometryCommand(String trajectoryName);

        /**
         * Creates a split odometry reset command.
         *
         * @param trajectoryName Choreo trajectory name
         * @param splitIndex split index
         * @return WPILib command
         */
        Command resetOdometryCommand(String trajectoryName, int splitIndex);

        /**
         * Creates a routine command.
         *
         * @param routineName Choreo routine name
         * @return WPILib command
         */
        Command routineCommand(String routineName);

        /**
         * Creates a Choreo cache warmup command.
         *
         * @return WPILib command
         */
        Command warmupCommand();
    }

    private static final class ChoreoFactoryClient implements FactoryClient {
        private final AutoFactory factory;

        private ChoreoFactoryClient(AutoFactory factory) {
            this.factory = Objects.requireNonNull(factory, "factory");
        }

        @Override
        public Command trajectoryCommand(String trajectoryName) {
            return factory.trajectoryCmd(trajectoryName);
        }

        @Override
        public Command trajectoryCommand(String trajectoryName, int splitIndex) {
            return factory.trajectoryCmd(trajectoryName, splitIndex);
        }

        @Override
        public Command resetOdometryCommand(String trajectoryName) {
            return factory.resetOdometry(trajectoryName);
        }

        @Override
        public Command resetOdometryCommand(String trajectoryName, int splitIndex) {
            return factory.resetOdometry(trajectoryName, splitIndex);
        }

        @Override
        public Command routineCommand(String routineName) {
            return factory.newRoutine(routineName).cmd();
        }

        @Override
        public Command warmupCommand() {
            return factory.warmupCmd();
        }
    }
}
