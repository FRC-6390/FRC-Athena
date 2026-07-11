package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.Autos;
import com.pathplanner.lib.auto.NamedCommands;

/** Level 2: PathPlanner event markers invoke named mechanism actions. */
public final class PathPlannerMarkersAuto {
    private PathPlannerMarkersAuto() {
    }

    /** Register before any PathPlannerAuto with these markers is constructed. */
    public static void registerMarkers(AutoContext context) {
        NamedCommands.registerCommand("pp-prepare-score", context.mechanisms.prepareToScore());
        NamedCommands.registerCommand("pp-score", context.mechanisms.score());
        NamedCommands.registerCommand("pp-intake", context.mechanisms.intakeUntilCaptured());
        NamedCommands.registerCommand("pp-stow", context.mechanisms.stow());
    }

    public static AutoRoutine create(AutoContext context) {
        return Autos.path(
                "PP 2 - Markers and actions",
                context.pathPlanner,
                "PP-Markers-And-States");
    }
}
