package frc.robot.auto;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Paths;

/** Selectable generated path with provider-owned execution and preview geometry. */
public final class GeneratedPathAuto {
    private GeneratedPathAuto() {
    }

    public static Action create() {
        return Paths.of(GeneratedPathProvider.KEY, "Straight-2m")
                .seconds(2.0)
                .resetOdometry();
    }
}
