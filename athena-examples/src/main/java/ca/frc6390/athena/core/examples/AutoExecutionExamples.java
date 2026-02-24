package ca.frc6390.athena.core.examples;

import ca.frc6390.athena.core.RobotAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.Optional;

/**
 * Example setup for chooser-based autonomous selection and execution.
 */
public final class AutoExecutionExamples {
    private AutoExecutionExamples() {}

    public static RobotAuto createWithTwoCustomPrograms(List<String> events) {
        RobotAuto autos = new RobotAuto();
        autos.registry().routine("safe", () -> Commands.runOnce(() -> events.add("safe")));
        autos.registry().routine("score", () -> Commands.runOnce(() -> events.add("score")));
        return autos;
    }

    public static void configureProgramChooser(RobotAuto autos, String defaultId) {
        autos.selection().programChooser(RobotAuto.AutoKey.of(defaultId));
    }

    public static void configureCommandChooser(RobotAuto autos, String defaultId) {
        autos.selection().commandChooser(RobotAuto.AutoKey.of(defaultId));
    }

    public static Optional<Command> selectedCommand(RobotAuto autos) {
        return autos.execution().selectedCommand();
    }
}
