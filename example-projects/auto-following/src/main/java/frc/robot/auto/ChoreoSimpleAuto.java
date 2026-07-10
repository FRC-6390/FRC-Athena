package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.Autos;

/** Level 1: run one Choreo trajectory through Athena's Choreo adapter. */
public final class ChoreoSimpleAuto {
    private ChoreoSimpleAuto() {
    }

    public static AutoRoutine create(AutoContext context) {
        return Autos.path(
                "Choreo 1 - Leave",
                context.choreo,
                "Choreo-Leave");
    }
}
