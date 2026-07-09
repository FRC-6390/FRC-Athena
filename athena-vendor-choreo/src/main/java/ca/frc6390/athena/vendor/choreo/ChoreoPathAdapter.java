package ca.frc6390.athena.vendor.choreo;

import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;

import ca.frc6390.athena.auto.PathProvider;
import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.PathAction;
import ca.frc6390.athena.mechanism.core.Paths;
import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Athena-facing adapter around Choreo's {@link AutoFactory} command API.
 */
public final class ChoreoPathAdapter implements PathProvider {
    /**
     * Source key used by Choreo path Actions.
     */
    public static final String KEY = "choreo";

    private final FactoryClient client;

    /**
     * Creates an adapter backed by a real Choreo {@link AutoFactory}.
     *
     * @param factory configured Choreo auto factory
     */
    public ChoreoPathAdapter(AutoFactory factory) {
        this(new ChoreoFactoryClient(factory));
    }

    /**
     * Creates an adapter backed by a custom client.
     *
     * @param client command-loading client
     */
    public ChoreoPathAdapter(FactoryClient client) {
        this.client = Objects.requireNonNull(client, "client");
    }

    /**
     * Creates a command that runs a Choreo trajectory.
     *
     * @param pathName Choreo trajectory name
     * @return WPILib command from the backing factory
     */
    public Command trajectoryCommand(String pathName) {
        return client.trajectoryCommand(normalize(pathName));
    }

    @Override
    public PathAction path(String pathName) {
        return Paths.choreo(normalize(pathName));
    }

    @Override
    public CommandAction load(String pathName) {
        String normalized = normalize(pathName);
        Command command = trajectoryCommand(normalized);
        return CommandAction.create(KEY + ":" + normalized)
                .onInitialize(command::initialize)
                .onExecute(command::execute)
                .until(command::isFinished)
                .onEnd(() -> command.end(false))
                .build();
    }

    /**
     * Creates an Athena path runtime that runs Choreo trajectories by path name.
     *
     * @return path runtime
     */
    public PathRuntime trajectoryRuntime() {
        return new CommandPathRuntime(path -> trajectoryCommand(path.name()));
    }

    @Override
    public PathRuntime runtime() {
        return trajectoryRuntime();
    }

    /**
     * Creates a command that runs one split of a Choreo trajectory.
     *
     * @param pathName Choreo trajectory name
     * @param splitIndex split index to run
     * @return WPILib command from the backing factory
     */
    public Command trajectoryCommand(String pathName, int splitIndex) {
        return client.trajectoryCommand(normalize(pathName), splitIndex);
    }

    /**
     * Creates a command that resets odometry from a Choreo trajectory.
     *
     * @param pathName Choreo trajectory name
     * @return WPILib command from the backing factory
     */
    public Command resetOdometryCommand(String pathName) {
        return client.resetOdometryCommand(normalize(pathName));
    }

    /**
     * Creates a command that resets odometry from one split of a Choreo trajectory.
     *
     * @param pathName Choreo trajectory name
     * @param splitIndex split index to read
     * @return WPILib command from the backing factory
     */
    public Command resetOdometryCommand(String pathName, int splitIndex) {
        return client.resetOdometryCommand(normalize(pathName), splitIndex);
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
     * Creates an Athena path runtime that runs Choreo routines by path name.
     *
     * @return path runtime
     */
    public PathRuntime routineRuntime() {
        return new CommandPathRuntime(path -> routineCommand(path.name()));
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

    private interface CommandFactory {
        Command command(PathAction path);
    }

    private static final class CommandPathRuntime implements PathRuntime {
        private final CommandFactory commandFactory;
        private final Map<String, Command> activeCommands = new ConcurrentHashMap<>();

        private CommandPathRuntime(CommandFactory commandFactory) {
            this.commandFactory = Objects.requireNonNull(commandFactory, "commandFactory");
        }

        @Override
        public void initialize(PathAction path, MechanismContext context) {
            Command command = commandFactory.command(path);
            activeCommands.put(path.key(), command);
            command.initialize();
        }

        @Override
        public void execute(PathAction path, MechanismContext context) {
            activeCommand(path).execute();
        }

        @Override
        public boolean isFinished(PathAction path, MechanismContext context) {
            return activeCommand(path).isFinished();
        }

        @Override
        public void end(PathAction path, MechanismContext context, boolean interrupted) {
            Command command = activeCommands.remove(path.key());
            if (command != null) {
                command.end(interrupted);
            }
        }

        private Command activeCommand(PathAction path) {
            return activeCommands.computeIfAbsent(path.key(), key -> commandFactory.command(path));
        }
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
