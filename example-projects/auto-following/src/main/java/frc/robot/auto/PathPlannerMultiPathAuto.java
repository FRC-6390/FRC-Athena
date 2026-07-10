package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.Autos;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Level 3: compose several PathPlanner .auto files and mechanism states in Java. */
public final class PathPlannerMultiPathAuto {
    private PathPlannerMultiPathAuto() {
    }

    public static AutoRoutine create(AutoContext context) {
        return Autos.routine("PP 3 - Java multipath", () -> {
            Command command = Commands.sequence(
                    context.mechanisms.score(),
                    context.pathPlanner.command("PP-To-Center-Piece"),
                    context.mechanisms.intakeUntilCaptured(),
                    context.pathPlanner.command("PP-Center-To-Score"),
                    context.mechanisms.score(),
                    context.pathPlanner.command("PP-Exit"));
            return ExampleCommands.fromWpilib("pp-java-multipath", command);
        });
    }
}
