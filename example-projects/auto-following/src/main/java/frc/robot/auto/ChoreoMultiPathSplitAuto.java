package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.wpilib.commands.WpilibCommands;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.concurrent.atomic.AtomicBoolean;

/** Level 3: chain multiple trajectories, including two splits from one Choreo path. */
public final class ChoreoMultiPathSplitAuto {
    private ChoreoMultiPathSplitAuto() {
    }

    public static AutoRoutine create(AutoContext context) {
        return Autos.routine("Choreo 3 - Multipath and splits", () -> {
            AtomicBoolean finished = new AtomicBoolean();
            choreo.auto.AutoRoutine routine =
                    context.choreoFactory.newRoutine("choreo-multipath-splits");

            AutoTrajectory outbound = routine.trajectory("Choreo-Two-Piece", 0);
            AutoTrajectory inbound = routine.trajectory("Choreo-Two-Piece", 1);
            AutoTrajectory exit = routine.trajectory("Choreo-Exit");

            routine.active().onTrue(outbound.resetOdometry().andThen(outbound.cmd()));
            outbound.atTime("deploy-intake").onTrue(context.mechanisms.intakeUntilCaptured());
            outbound.done().onTrue(inbound.cmd());
            inbound.atTimeBeforeEnd(0.35).onTrue(context.mechanisms.prepareToScore());
            inbound.done().onTrue(context.mechanisms.score().andThen(exit.cmd()));
            exit.done().onTrue(Commands.runOnce(() -> finished.set(true)));

            return WpilibCommands.wrap(routine.cmd(finished::get).withName("choreo-multipath-splits"));
        });
    }
}
