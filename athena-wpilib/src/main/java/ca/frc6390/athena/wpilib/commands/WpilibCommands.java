package ca.frc6390.athena.wpilib.commands;

import ca.frc6390.athena.commands.CommandAction;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.util.Objects;

/**
 * Explicit interoperability between Athena actions and WPILib commands.
 */
public final class WpilibCommands {
    private WpilibCommands() {
    }

    public static CommandAction run(String name, Runnable action) {
        Objects.requireNonNull(action, "action");
        return CommandAction.create(name)
                .onInitialize(action::run)
                .until(() -> true)
                .build();
    }

    public static CommandAction loop(String name, Runnable action) {
        Objects.requireNonNull(action, "action");
        return CommandAction.create(name)
                .onExecute(action::run)
                .build();
    }

    public static CommandAction wrap(Command command) {
        Objects.requireNonNull(command, "command");
        CommandScheduler scheduler = CommandScheduler.getInstance();
        return CommandAction.create(command.getName())
                .onInitialize(() -> scheduler.schedule(command))
                .until(() -> !scheduler.isScheduled(command))
                .onEnd(() -> scheduler.cancel(command))
                .build();
    }
}
