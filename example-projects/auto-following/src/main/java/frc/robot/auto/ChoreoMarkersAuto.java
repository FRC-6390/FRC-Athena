package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.mechanism.core.PathAction;

/** Choreo markers run ordinary Athena mechanism Actions. */
public final class ChoreoMarkersAuto {
    private ChoreoMarkersAuto() {
    }

    public static AutoRoutine create(AutoContext context) {
        PathAction path = context.choreo.path("Choreo-Markers-And-States")
                .resetOdometry()
                .marker("choreo-prepare-score", context.mechanisms.prepareToScore)
                .marker("choreo-score", context.mechanisms.score)
                .marker("choreo-intake", context.mechanisms.intakeUntilCaptured)
                .marker("choreo-stow", context.mechanisms.stow);
        return Autos.routine("Choreo 1 - Markers", path);
    }
}
