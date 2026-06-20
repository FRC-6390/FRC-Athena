package ca.frc6390.athena.examples;

import ca.frc6390.athena.commands.CommandGroups;
import ca.frc6390.athena.commands.CommandSpec;

/**
 * Example command declarations built without depending directly on WPILib.
 */
public final class RobotCommands {
    private RobotCommands() {
    }

    /**
     * Creates an intake command descriptor.
     *
     * @param intakeRunning callback invoked while the intake is running
     * @return command spec
     */
    public static CommandSpec runIntake(Runnable intakeRunning) {
        return CommandSpec.create("runIntake")
                .onExecute(intakeRunning::run)
                .toSpec();
    }

    /**
     * Creates a simple score sequence using composed command specs.
     *
     * @param spinShooter callback invoked while the shooter spins up
     * @param feedNote callback invoked while the note is fed
     * @return composed command spec
     */
    public static CommandSpec scoreSequence(Runnable spinShooter, Runnable feedNote) {
        CommandSpec spinUp = CommandSpec.create("spinShooter")
                .onExecute(spinShooter::run)
                .until(() -> true)
                .toSpec();
        CommandSpec feed = CommandSpec.create("feedNote")
                .onExecute(feedNote::run)
                .until(() -> true)
                .toSpec();
        return CommandGroups.sequence("scoreSequence", spinUp, feed);
    }
}
