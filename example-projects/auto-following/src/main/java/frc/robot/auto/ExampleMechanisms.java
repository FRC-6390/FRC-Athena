package frc.robot.auto;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;

/** Replace these print-only Actions with real mechanism Actions. */
public final class ExampleMechanisms {
    public final Action prepareToScore;
    public final Action score;
    public final Action intakeUntilCaptured;
    public final Action stow;

    public ExampleMechanisms(ExampleRobotState state) {
        prepareToScore = Actions.doOnce(() -> System.out.println("prepare score"));
        score = Actions.sequence()
                .run(Actions.doOnce(() -> System.out.println("score")))
                .then(Actions.waitSeconds(0.20))
                .then(Actions.doOnce(() -> state.setHasGamePiece(false)));
        intakeUntilCaptured = Actions.timeout(
                Actions.waitUntil(state::hasGamePiece), 1.5);
        stow = Actions.doOnce(() -> System.out.println("stow"));
    }
}
