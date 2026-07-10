package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.AutoRuntime;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.auto.PathGraph;
import java.util.List;

/** Registry only; each routine's implementation lives in its own file. */
public final class AutoExamples {
    public final AutoContext context;
    public final TeleopAssistRoutines teleopAssists;
    public final AutoRuntime runtime;
    public final PathGraph customMarkers;

    public AutoExamples(ExampleDrive drive) {
        context = new AutoContext(drive);
        teleopAssists = new TeleopAssistRoutines(context);

        PathPlannerMarkersAuto.registerMarkers(context);
        ChoreoMarkersAuto.registerMarkers(context);

        ExampleMarkerPathProvider customProvider = new ExampleMarkerPathProvider();
        AutoRoutine customProviderAuto = CustomProviderMarkersAuto.create(context, customProvider);
        List<AutoRoutine> routines = List.of(
                PathPlannerSimpleAuto.create(context),
                PathPlannerMarkersAuto.create(context),
                PathPlannerMultiPathAuto.create(context),
                ConditionalPathPlannerAuto.create(context),
                DynamicPathPlannerAuto.create(context),
                ChoreoSimpleAuto.create(context),
                ChoreoMarkersAuto.create(context),
                ChoreoMultiPathSplitAuto.create(context),
                ConditionalChoreoAuto.create(context),
                customProviderAuto);

        runtime = Autos.runtime(routines);
        customMarkers = PathGraph.of(customProviderAuto);
        customProvider.markers(customMarkers::trigger);
    }
}
