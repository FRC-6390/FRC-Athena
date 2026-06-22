package ca.frc6390.athena.wpilib.commands;

import java.util.Objects;
import java.util.Map;

import ca.frc6390.athena.commands.CommandSpec;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Adapts Athena command specs to real WPILib command objects.
 */
public final class WpilibCommandAdapter {
    private WpilibCommandAdapter() {
    }

    /**
     * Creates a lifecycle object from an Athena command spec.
     *
     * @param spec command spec
     * @return WPILib command
     */
    public static Command adapt(CommandSpec spec) {
        return adapt(spec, Map.of());
    }

    /**
     * Creates a lifecycle object from an Athena command spec and requirement map.
     *
     * @param spec command spec
     * @param subsystems subsystems keyed by Athena requirement name
     * @return WPILib command
     */
    public static Command adapt(CommandSpec spec, Map<String, ? extends Subsystem> subsystems) {
        Objects.requireNonNull(spec, "spec");
        Objects.requireNonNull(subsystems, "subsystems");
        return new AthenaCommand(spec, subsystems);
    }

    private static final class AthenaCommand extends Command {
        private final CommandSpec spec;

        private AthenaCommand(CommandSpec spec, Map<String, ? extends Subsystem> subsystems) {
            this.spec = spec;
            setName(spec.name());
            addRequirements(spec.requirements().stream()
                    .map(requirement -> requireSubsystem(requirement, subsystems))
                    .toList());
        }

        @Override
        public void initialize() {
            spec.onInitialize().run();
        }

        @Override
        public void execute() {
            spec.onExecute().run();
        }

        @Override
        public boolean isFinished() {
            return spec.isFinished().get();
        }

        @Override
        public void end(boolean interrupted) {
            spec.onEnd().run();
        }

        private static Subsystem requireSubsystem(String requirement, Map<String, ? extends Subsystem> subsystems) {
            Subsystem subsystem = subsystems.get(requirement);
            if (subsystem == null) {
                throw new IllegalArgumentException("No WPILib subsystem provided for Athena requirement " + requirement + ".");
            }
            return subsystem;
        }
    }
}
