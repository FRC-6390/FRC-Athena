package ca.frc6390.athena.wpilib.commands;

import ca.frc6390.athena.commands.CommandSpec;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Map;
import java.util.Objects;

/**
 * Convenience scheduler facade for Athena command specs.
 */
public final class WpilibCommandScheduler {
    private final SchedulerClient scheduler;

    /**
     * Creates a scheduler facade backed by WPILib's global scheduler.
     */
    public WpilibCommandScheduler() {
        this(new WpilibSchedulerClient(CommandScheduler.getInstance()));
    }

    /**
     * Creates a scheduler facade backed by a custom scheduler client.
     *
     * @param scheduler scheduler client
     */
    public WpilibCommandScheduler(SchedulerClient scheduler) {
        this.scheduler = Objects.requireNonNull(scheduler, "scheduler");
    }

    /**
     * Adapts and schedules an Athena command spec.
     *
     * @param spec command spec
     * @return scheduled WPILib command
     */
    public Command schedule(CommandSpec spec) {
        return schedule(spec, Map.of());
    }

    /**
     * Adapts and schedules an Athena command spec with named subsystem requirements.
     *
     * @param spec command spec
     * @param subsystems subsystems keyed by Athena requirement name
     * @return scheduled WPILib command
     */
    public Command schedule(CommandSpec spec, Map<String, ? extends Subsystem> subsystems) {
        Command command = WpilibCommandAdapter.adapt(spec, subsystems);
        scheduler.schedule(command);
        return command;
    }

    /**
     * Runs one scheduler loop.
     */
    public void run() {
        scheduler.run();
    }

    /**
     * Cancels a scheduled command.
     *
     * @param command command to cancel
     */
    public void cancel(Command command) {
        scheduler.cancel(command);
    }

    /**
     * Minimal scheduler operations used by the facade.
     */
    public interface SchedulerClient {
        /**
         * Schedules a command.
         *
         * @param command command
         */
        void schedule(Command command);

        /**
         * Runs one scheduler loop.
         */
        void run();

        /**
         * Cancels a command.
         *
         * @param command command
         */
        void cancel(Command command);
    }

    private record WpilibSchedulerClient(CommandScheduler scheduler) implements SchedulerClient {
        private WpilibSchedulerClient {
            Objects.requireNonNull(scheduler, "scheduler");
        }

        @Override
        public void schedule(Command command) {
            scheduler.schedule(command);
        }

        @Override
        public void run() {
            scheduler.run();
        }

        @Override
        public void cancel(Command command) {
            scheduler.cancel(command);
        }
    }
}
