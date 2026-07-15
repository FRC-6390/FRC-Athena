package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.AutoRuntime;
import java.util.List;

/** Registry only; each routine's implementation lives in its own file. */
public final class AutoExamples {
    public final AutoContext context;
    public final AutoRuntime runtime;

    public AutoExamples(ExampleDrive drive) {
        context = new AutoContext(drive);
        List<AutoRoutine> routines = List.of(
                ChoreoMarkersAuto.create(context),
                ChoreoMultiPathSplitAuto.create(context),
                ConditionalChoreoAuto.create(context));

        runtime = new AutoRuntime(routines);
    }
}
