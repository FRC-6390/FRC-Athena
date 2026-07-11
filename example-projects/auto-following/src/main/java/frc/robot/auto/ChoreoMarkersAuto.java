package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.Autos;

/** Level 2: Choreo event markers invoke factory-wide mechanism actions. */
public final class ChoreoMarkersAuto {
    private ChoreoMarkersAuto() {
    }

    /** Bind before creating trajectories that should receive these events. */
    public static void registerMarkers(AutoContext context) {
        context.choreoFactory
                .bind("choreo-prepare-score", context.mechanisms.prepareToScore())
                .bind("choreo-score", context.mechanisms.score())
                .bind("choreo-intake", context.mechanisms.intakeUntilCaptured())
                .bind("choreo-stow", context.mechanisms.stow());
    }

    public static AutoRoutine create(AutoContext context) {
        return Autos.path(
                "Choreo 2 - Markers and actions",
                context.choreo,
                "Choreo-Markers-And-States");
    }
}
