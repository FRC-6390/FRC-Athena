package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.Autos;

/** Level 1: run one PathPlanner .auto file. */
public final class PathPlannerSimpleAuto {
    private PathPlannerSimpleAuto() {
    }

    public static AutoRoutine create(AutoContext context) {
        return Autos.path(
                "PP 1 - Leave",
                context.pathPlanner,
                "PP-Leave");
    }
}
