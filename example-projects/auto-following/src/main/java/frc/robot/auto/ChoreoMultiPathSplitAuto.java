package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;

/** Level 3: chain multiple trajectories, including two splits from one Choreo path. */
public final class ChoreoMultiPathSplitAuto {
    private ChoreoMultiPathSplitAuto() {
    }

    public static AutoRoutine create(AutoContext context) {
        Action action = Actions.sequence()
                .run(context.choreo.split("Choreo-Two-Piece", 0).resetOdometry())
                .then(Actions.deadline(
                        context.choreo.split("Choreo-Two-Piece", 1),
                        context.mechanisms.prepareToScore))
                .then(context.mechanisms.score)
                .then(context.choreo.path("Choreo-Exit"));
        return Autos.routine("Choreo 2 - Multipath and splits", action);
    }
}
