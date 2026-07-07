package ca.frc6390.athena.vendor.pathplanner;

import ca.frc6390.athena.auto.AutoSource;
import ca.frc6390.athena.commands.CommandSpec;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.wpilib.commands.WpilibCommandPathRuntime;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.List;
import java.util.Objects;

/**
 * PathPlanner autonomous source backed by PathPlannerLib.
 */
public final class PathPlannerAutoSource implements AutoSource {
    /**
     * Source key used in Athena auto declarations.
     */
    public static final String KEY = "pathplanner";

    private final PathPlannerClient client;

    /**
     * Creates a source that delegates to PathPlannerLib's global AutoBuilder.
     */
    public PathPlannerAutoSource() {
        this(new AutoBuilderPathPlannerClient());
    }

    PathPlannerAutoSource(PathPlannerClient client) {
        this.client = Objects.requireNonNull(client, "client");
    }

    @Override
    public CommandSpec load(String routinePath) {
        String path = routinePath == null || routinePath.isBlank() ? "default" : routinePath.trim();
        return CommandSpec.create(KEY + ":" + path).toSpec();
    }

    /**
     * Loads a real WPILib command from a PathPlanner auto file.
     *
     * @param routinePath PathPlanner auto name
     * @return WPILib command
     */
    public Command loadCommand(String routinePath) {
        return client.buildAuto(normalize(routinePath));
    }

    /**
     * Creates an Athena path runtime that runs PathPlanner autos by path name.
     *
     * @return path runtime
     */
    public PathRuntime runtime() {
        return WpilibCommandPathRuntime.of(path -> loadCommand(path.name()));
    }

    /**
     * Returns auto names discovered by PathPlannerLib.
     *
     * @return auto names
     */
    public List<String> autoNames() {
        return client.autoNames();
    }

    private static String normalize(String routinePath) {
        return routinePath == null || routinePath.isBlank() ? "default" : routinePath.trim();
    }

    interface PathPlannerClient {
        Command buildAuto(String autoName);

        List<String> autoNames();
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
