package ca.frc6390.athena.vendor.pathplanner;

import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;

import ca.frc6390.athena.auto.PathProvider;
import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.PathAction;
import ca.frc6390.athena.mechanism.core.Paths;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * PathPlanner-backed provider for Athena path Actions and runtimes.
 */
public final class PathPlannerPathProvider implements PathProvider {
    /**
     * Source key used by PathPlanner path Actions.
     */
    public static final String KEY = "pathplanner";

    private final PathPlannerClient client;
    private final Map<String, PathAction> pathCache = new ConcurrentHashMap<>();
    private volatile List<String> pathNameCache;

    /**
     * Creates a provider that delegates to PathPlannerLib's global AutoBuilder.
     */
    public PathPlannerPathProvider() {
        this(new AutoBuilderPathPlannerClient());
    }

    PathPlannerPathProvider(PathPlannerClient client) {
        this.client = Objects.requireNonNull(client, "client");
    }

    /**
     * Creates a PathPlanner path Action.
     *
     * @param pathName PathPlanner auto name
     * @return path Action
     */
    public PathAction path(String pathName) {
        return pathCache.computeIfAbsent(normalize(pathName), Paths::pathPlanner);
    }

    @Override
    public CommandAction load(String pathName) {
        String normalized = normalize(pathName);
        Command command = command(normalized);
        return CommandAction.create(KEY + ":" + normalized)
                .onInitialize(command::initialize)
                .onExecute(command::execute)
                .until(command::isFinished)
                .onEnd(() -> command.end(false))
                .build();
    }

    /**
     * Loads a real WPILib command from a PathPlanner auto file.
     *
     * @param pathName PathPlanner auto name
     * @return WPILib command
     */
    public Command command(String pathName) {
        return client.buildAuto(normalize(pathName));
    }

    /**
     * Creates an Athena path runtime that runs PathPlanner autos by path name.
     *
     * @return path runtime
     */
    public PathRuntime runtime() {
        return new CommandPathRuntime(path -> command(path.name()));
    }

    /**
     * Returns path names discovered by PathPlannerLib.
     *
     * @return path names
     */
    public List<String> pathNames() {
        List<String> names = pathNameCache;
        if (names == null) {
            names = client.autoNames();
            pathNameCache = names;
        }
        return names;
    }

    private static String normalize(String pathName) {
        return pathName == null || pathName.isBlank() ? "default" : pathName.trim();
    }

    interface PathPlannerClient {
        Command buildAuto(String autoName);

        List<String> autoNames();
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

    private static final class AutoBuilderPathPlannerClient implements PathPlannerClient {
        @Override
        public Command buildAuto(String autoName) {
            return AutoBuilder.buildAuto(autoName);
        }

        @Override
        public List<String> autoNames() {
            return List.copyOf(AutoBuilder.getAllAutoNames());
        }
    }
}
