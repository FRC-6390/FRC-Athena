package ca.frc6390.athena.wpilib.commands;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.LifecycleAction;
import ca.frc6390.athena.mechanism.core.NamedResource;
import ca.frc6390.athena.robot.RobotRuntime;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.Objects;
import java.util.Set;
import java.util.WeakHashMap;
import java.util.stream.Collectors;

/**
 * Two-way interoperability between Athena Actions and WPILib commands.
 *
 * <ul>
 *   <li>{@link #action(Command)} wraps a WPILib command as a lifecycle Action. The command runs in WPILib's
 *       scheduler; the Action completes when the command finishes and ends the command when interrupted.
 *       Each required subsystem becomes a stable {@link NamedResource}, so two wrapped commands that share a
 *       subsystem exclude each other through the Athena lease system.</li>
 *   <li>{@link #command(RobotRuntime, Action)} wraps an Athena Action as a WPILib command that requests the
 *       Action from the robot runtime, finishes when it completes, and cancels it when interrupted.</li>
 * </ul>
 */
public final class WpilibCommands {
    private static final WeakHashMap<Subsystem, NamedResource> SUBSYSTEM_RESOURCES = new WeakHashMap<>();

    private WpilibCommands() {
    }

    /**
     * Wraps a WPILib command as an Athena lifecycle Action.
     *
     * @param command WPILib command
     * @return lifecycle Action that schedules the command when it starts
     */
    public static LifecycleAction action(Command command) {
        Objects.requireNonNull(command, "command");
        CommandScheduler scheduler = CommandScheduler.getInstance();
        Object[] requirements = command.getRequirements().stream()
                .map(WpilibCommands::resource)
                .toArray();
        return Actions.lifecycle(command.getName())
                .onStart(() -> scheduler.schedule(command))
                .until(() -> !scheduler.isScheduled(command))
                .onEnd(interrupted -> scheduler.cancel(command))
                .requires(requirements)
                .build();
    }

    /**
     * Wraps an Athena Action as a WPILib command. Scheduling the command requests the Action from the
     * runtime; the command finishes when the Action completes and cancels it when the command is
     * interrupted.
     *
     * @param runtime robot runtime that owns the Action
     * @param action Action to request
     * @return WPILib command
     */
    public static Command command(RobotRuntime runtime, Action action) {
        return new ActionCommand(
                Objects.requireNonNull(runtime, "runtime"),
                Objects.requireNonNull(action, "action"));
    }

    /**
     * Returns the stable Athena resource for a WPILib subsystem. The first subsystem that claims a name keeps
     * it; later subsystems with the same name get a numeric suffix, so distinct subsystems never share a lease.
     *
     * @param subsystem WPILib subsystem
     * @return named resource
     */
    public static synchronized NamedResource resource(Subsystem subsystem) {
        Objects.requireNonNull(subsystem, "subsystem");
        NamedResource existing = SUBSYSTEM_RESOURCES.get(subsystem);
        if (existing != null) return existing;
        String reported = subsystem.getName();
        if (reported == null || reported.isBlank()) reported = subsystem.getClass().getSimpleName();
        String base = reported.isBlank() ? "subsystem" : reported;
        Set<String> names = SUBSYSTEM_RESOURCES.values().stream()
                .map(NamedResource::name)
                .collect(Collectors.toSet());
        String name = base;
        for (int suffix = 2; names.contains("subsystem:" + name); suffix++) {
            name = base + "#" + suffix;
        }
        NamedResource resource = NamedResource.of("subsystem:" + name);
        SUBSYSTEM_RESOURCES.put(subsystem, resource);
        return resource;
    }

    private static final class ActionCommand extends Command {
        private final RobotRuntime runtime;
        private final Action action;
        private boolean started;

        private ActionCommand(RobotRuntime runtime, Action action) {
            this.runtime = runtime;
            this.action = action;
            setName("athena-action");
        }

        @Override
        public void initialize() {
            started = false;
            runtime.request(action);
        }

        @Override
        public boolean isFinished() {
            if (runtime.isActionComplete(action)) return true;
            boolean running = runtime.isActionRunning(action);
            if (running) started = true;
            // Held or not yet started requests keep the command alive; a request that was running and is
            // no longer running ended from outside (cancelled or replaced).
            return started && !running;
        }

        @Override
        public void end(boolean interrupted) {
            if (runtime.isActionRunning(action)) runtime.cancel(action);
        }
    }
}
