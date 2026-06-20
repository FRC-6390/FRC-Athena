package ca.frc6390.athena.wpilib.commands;

import ca.frc6390.athena.commands.CommandSpec;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.Map;
import java.util.Objects;

/**
 * Trigger binding helpers for Athena command specs.
 */
public final class WpilibTriggerBindings {
    private WpilibTriggerBindings() {
    }

    /**
     * Binds a command spec to run when a trigger becomes true.
     *
     * @param trigger WPILib trigger
     * @param spec command spec
     * @param subsystems subsystems keyed by Athena requirement name
     * @return adapted command
     */
    public static Command onTrue(
            Trigger trigger,
            CommandSpec spec,
            Map<String, ? extends Subsystem> subsystems) {
        return bind(new RealTriggerBinder(trigger), TriggerMode.ON_TRUE, spec, subsystems);
    }

    /**
     * Binds a command spec to run while a trigger is true.
     *
     * @param trigger WPILib trigger
     * @param spec command spec
     * @param subsystems subsystems keyed by Athena requirement name
     * @return adapted command
     */
    public static Command whileTrue(
            Trigger trigger,
            CommandSpec spec,
            Map<String, ? extends Subsystem> subsystems) {
        return bind(new RealTriggerBinder(trigger), TriggerMode.WHILE_TRUE, spec, subsystems);
    }

    /**
     * Binds a command spec to toggle when a trigger becomes true.
     *
     * @param trigger WPILib trigger
     * @param spec command spec
     * @param subsystems subsystems keyed by Athena requirement name
     * @return adapted command
     */
    public static Command toggleOnTrue(
            Trigger trigger,
            CommandSpec spec,
            Map<String, ? extends Subsystem> subsystems) {
        return bind(new RealTriggerBinder(trigger), TriggerMode.TOGGLE_ON_TRUE, spec, subsystems);
    }

    /**
     * Adapts and binds a command spec through a binder.
     *
     * @param binder trigger binder
     * @param mode binding mode
     * @param spec command spec
     * @param subsystems subsystems keyed by Athena requirement name
     * @return adapted command
     */
    public static Command bind(
            TriggerBinder binder,
            TriggerMode mode,
            CommandSpec spec,
            Map<String, ? extends Subsystem> subsystems) {
        Objects.requireNonNull(binder, "binder");
        TriggerMode bindingMode = mode == null ? TriggerMode.ON_TRUE : mode;
        Command command = WpilibCommandAdapter.adapt(spec, subsystems == null ? Map.of() : subsystems);
        switch (bindingMode) {
            case ON_TRUE -> binder.onTrue(command);
            case WHILE_TRUE -> binder.whileTrue(command);
            case TOGGLE_ON_TRUE -> binder.toggleOnTrue(command);
            default -> throw new IllegalStateException("Unhandled trigger binding mode " + bindingMode + ".");
        }
        return command;
    }

    /**
     * Supported trigger binding modes.
     */
    public enum TriggerMode {
        /**
         * Run once when the trigger becomes true.
         */
        ON_TRUE,
        /**
         * Run while the trigger remains true.
         */
        WHILE_TRUE,
        /**
         * Toggle whenever the trigger becomes true.
         */
        TOGGLE_ON_TRUE
    }

    /**
     * Minimal trigger binding surface.
     */
    public interface TriggerBinder {
        /**
         * Binds an on-true command.
         *
         * @param command command
         */
        void onTrue(Command command);

        /**
         * Binds a while-true command.
         *
         * @param command command
         */
        void whileTrue(Command command);

        /**
         * Binds a toggle-on-true command.
         *
         * @param command command
         */
        void toggleOnTrue(Command command);
    }

    private record RealTriggerBinder(Trigger trigger) implements TriggerBinder {
        private RealTriggerBinder {
            Objects.requireNonNull(trigger, "trigger");
        }

        @Override
        public void onTrue(Command command) {
            trigger.onTrue(command);
        }

        @Override
        public void whileTrue(Command command) {
            trigger.whileTrue(command);
        }

        @Override
        public void toggleOnTrue(Command command) {
            trigger.toggleOnTrue(command);
        }
    }
}
