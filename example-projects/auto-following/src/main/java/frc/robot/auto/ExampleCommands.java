package frc.robot.auto;

import ca.frc6390.athena.commands.CommandAction;

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
}
