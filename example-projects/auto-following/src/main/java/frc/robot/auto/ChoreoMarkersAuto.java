package frc.robot.auto;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Paths;

/** Choreo markers run ordinary Athena mechanism Actions. */
public final class ChoreoMarkersAuto {
    private ChoreoMarkersAuto() {
    }

    public static Action create(AutoContext context) {
        return Paths.choreo("Choreo-Markers-And-States")
                .resetOdometry()
                .marker("choreo-prepare-score", context.mechanisms.prepareToScore)
                .marker("choreo-score", context.mechanisms.score)
                .marker("choreo-intake", context.mechanisms.intakeUntilCaptured)
                .marker("choreo-stow", context.mechanisms.stow);
    }
}
