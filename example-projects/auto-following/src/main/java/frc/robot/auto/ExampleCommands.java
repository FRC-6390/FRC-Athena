package frc.robot.auto;

import ca.frc6390.athena.commands.CommandAction;
import edu.wpi.first.wpilibj2.command.Command;

public final class ExampleCommands {
    private ExampleCommands() {
    }

    public static CommandAction command(String name) {
        return CommandAction.create(name)
                .onInitialize(() -> {})
                .onExecute(() -> {})
                .until(() -> false)
                .onEnd(() -> {})
                .build();
    }

    /** Adapts a WPILib command so it can be selected by Athena's auto runtime. */
    public static CommandAction fromWpilib(String name, Command command) {
        return CommandAction.create(name)
                .onInitialize(command::initialize)
                .onExecute(command::execute)
                .until(command::isFinished)
                .onEnd(() -> command.end(false))
                .build();
    }
}
