package frc.robot.auto;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Paths;

/** Level 4: select the next Choreo path from sensor state at the branch point. */
public final class ConditionalChoreoAuto {
    private ConditionalChoreoAuto() {
    }

    public static Action create(AutoContext context) {
        Action returnAndScore = Actions.sequence()
                .run(Paths.choreo("Choreo-Return-To-Score"))
                .then(context.mechanisms.score);
        return Actions.sequence()
                .run(Paths.choreo("Choreo-Collect").resetOdometry())
                .then(Actions.when(context.state::hasGamePiece)
                        .run(returnAndScore)
                        .otherwise(Paths.choreo("Choreo-Safe-Exit")));
    }
}
