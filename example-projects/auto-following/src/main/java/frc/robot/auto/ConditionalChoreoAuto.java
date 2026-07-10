package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.Autos;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.concurrent.atomic.AtomicBoolean;

/** Level 4: select the next Choreo path from sensor state at the branch point. */
public final class ConditionalChoreoAuto {
    private ConditionalChoreoAuto() {
    }

    public static AutoRoutine create(AutoContext context) {
        return Autos.routine("Choreo 4 - Conditional branch", () -> {
            AtomicBoolean finished = new AtomicBoolean();
            choreo.auto.AutoRoutine routine =
                    context.choreoFactory.newRoutine("choreo-conditional");
            AutoTrajectory collect = routine.trajectory("Choreo-Collect");
            AutoTrajectory returnToScore = routine.trajectory("Choreo-Return-To-Score");
            AutoTrajectory safeExit = routine.trajectory("Choreo-Safe-Exit");

            routine.active().onTrue(collect.resetOdometry().andThen(collect.cmd()));
            collect.done().onTrue(Commands.either(
                    returnToScore.cmd().andThen(context.mechanisms.score()),
                    safeExit.cmd(),
                    context.state::hasGamePiece).andThen(Commands.runOnce(() -> finished.set(true))));

            return ExampleCommands.fromWpilib(
                    "choreo-conditional",
                    routine.cmd(finished::get));
        });
    }
}
