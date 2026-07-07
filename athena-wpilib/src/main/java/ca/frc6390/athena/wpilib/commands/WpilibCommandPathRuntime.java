package ca.frc6390.athena.wpilib.commands;

import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.PathRef;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Objects;
import java.util.function.Function;

/**
 * Runs WPILib commands through Athena path-state lifecycle.
 */
public final class WpilibCommandPathRuntime implements PathRuntime {
    private final Function<PathRef, Command> commandFactory;
    private Command command;

    private WpilibCommandPathRuntime(Function<PathRef, Command> commandFactory) {
        this.commandFactory = Objects.requireNonNull(commandFactory, "commandFactory");
    }

    /**
     * Creates a runtime from a command factory.
     *
     * @param commandFactory command factory for each path ref
     * @return path runtime
     */
    public static PathRuntime of(Function<PathRef, Command> commandFactory) {
        return new WpilibCommandPathRuntime(commandFactory);
    }

    @Override
    public void initialize(PathRef path, MechanismContext context) {
        command = Objects.requireNonNull(commandFactory.apply(path), "command");
        command.initialize();
    }

    @Override
    public void execute(PathRef path, MechanismContext context) {
        if (command != null) {
            command.execute();
        }
    }

    @Override
    public boolean isFinished(PathRef path, MechanismContext context) {
        return command == null || command.isFinished();
    }

    @Override
    public void end(PathRef path, MechanismContext context, boolean interrupted) {
        if (command != null) {
            command.end(interrupted);
            command = null;
        }
    }
}
