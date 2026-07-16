package frc.robot.auto;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Paths;

/** Level 3: chain multiple trajectories, including two splits from one Choreo path. */
public final class ChoreoMultiPathSplitAuto {
    private ChoreoMultiPathSplitAuto() {
    }

    public static Action create(AutoContext context) {
        return Actions.sequence()
                .run(Paths.choreo("Choreo-Two-Piece").split(0).resetOdometry())
                .then(Actions.deadline(
                        Paths.choreo("Choreo-Two-Piece").split(1),
                        context.mechanisms.prepareToScore))
                .then(context.mechanisms.score)
                .then(Paths.choreo("Choreo-Exit"));
    }
}
