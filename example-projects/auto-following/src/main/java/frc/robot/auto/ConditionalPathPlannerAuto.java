package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.Autos;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/** Level 4: make decisions when the auto reaches each branch, not at robot startup. */
public final class ConditionalPathPlannerAuto {
    private ConditionalPathPlannerAuto() {
    }

    public static AutoRoutine create(AutoContext context) {
        return Autos.routine("PP 4 - Conditional", () -> {
            Command scoreOrContinue = Commands.either(
                    context.mechanisms.score(),
                    context.mechanisms.stow(),
                    context.state::hasGamePiece);

            Command chooseLane = Commands.either(
                    context.pathPlanner.command("PP-Center-Lane"),
                    context.pathPlanner.command("PP-Safe-Lane"),
                    context.state::centerLaneClear);

            return ExampleCommands.fromWpilib(
                    "pp-conditional",
                    Commands.sequence(scoreOrContinue, chooseLane));
        });
    }
}
