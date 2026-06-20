package ca.frc6390.athena.superstructure.runtime;

import ca.frc6390.athena.commands.CommandSpec;
import java.util.Arrays;
import java.util.Objects;

/**
 * Command factories for applying superstructure states.
 */
public final class SuperstructureCommands {
    private SuperstructureCommands() {
    }

    /**
     * Creates a one-shot command that applies a target superstructure state.
     *
     * @param controller runtime superstructure controller
     * @param targetState target state name
     * @param requirements named subsystem or resource requirements
     * @return command spec
     */
    public static CommandSpec applyState(
            SuperstructureController controller,
            String targetState,
            String... requirements) {
        Objects.requireNonNull(controller, "controller");
        String stateName = targetState == null || targetState.isBlank() ? "default" : targetState.trim();
        return CommandSpec.create("superstructure:" + stateName)
                .requires(requirements == null ? java.util.List.of() : Arrays.asList(requirements))
                .onInitialize(() -> controller.applyState(stateName))
                .until(() -> true)
                .toSpec();
    }

    /**
     * Creates a command that stops all registered mechanism controllers.
     *
     * @param controller runtime superstructure controller
     * @param requirements named subsystem or resource requirements
     * @return command spec
     */
    public static CommandSpec stop(
            SuperstructureController controller,
            String... requirements) {
        Objects.requireNonNull(controller, "controller");
        return CommandSpec.create("superstructure:stop")
                .requires(requirements == null ? java.util.List.of() : Arrays.asList(requirements))
                .onInitialize(controller::stop)
                .until(() -> true)
                .toSpec();
    }
}
