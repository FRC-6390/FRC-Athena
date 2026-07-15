package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;

/** Level 4: select the next Choreo path from sensor state at the branch point. */
public final class ConditionalChoreoAuto {
    private ConditionalChoreoAuto() {
    }

    public static AutoRoutine create(AutoContext context) {
        Action returnAndScore = Actions.sequence()
                .run(context.choreo.path("Choreo-Return-To-Score"))
                .then(context.mechanisms.score);
        Action action = Actions.sequence()
                .run(context.choreo.path("Choreo-Collect").resetOdometry())
                .then(Actions.when(context.state::hasGamePiece)
                        .run(returnAndScore)
                        .otherwise(context.choreo.path("Choreo-Safe-Exit")));
        return Autos.routine("Choreo 3 - Conditional branch", action);
    }
}
