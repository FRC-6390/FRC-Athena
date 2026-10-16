package frc.robot.auto;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.wpilib.commands.WpilibCommands;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;

/** Explicit WPILib Command to Athena Action adaptation. */
public final class CommandInteropExamples implements Mechanism {
    public final Action announceTest = WpilibCommands.action(
            Commands.print("WPILib command scheduled through Athena"));
    public final HookBinding scheduleInTest;

    public CommandInteropExamples(Robot robot) {
        scheduleInTest = Events.testInit().onStart(() -> robot.athena().request(announceTest));
    }
}
