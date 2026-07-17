package frc.robot.auto;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Paths;

/** Runs a PathPlanner auto file through Athena's normal PathAction lifecycle. */
public final class PathPlannerAuto {
    private PathPlannerAuto() {
    }

    public static Action create() {
        return Paths.pathPlanner("PathPlanner-Two-Piece");
    }
}
