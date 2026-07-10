package frc.robot.auto;

import ca.frc6390.athena.auto.AutoRuntime;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.auto.PathGraph;
import ca.frc6390.athena.auto.PathProvider;
import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.mechanism.core.PathAction;
import ca.frc6390.athena.mechanism.core.Paths;
import ca.frc6390.athena.vendor.choreo.ChoreoPathProvider;
import ca.frc6390.athena.vendor.pathplanner.PathPlannerPathProvider;

public final class AutoExamples {
    private final PathPlannerPathProvider pathPlanner = new PathPlannerPathProvider();
    private final ChoreoPathProvider choreoFiles = new ChoreoPathProvider();
    private final PathProvider choreoFactoryStyle = new TimedPathProvider("choreo", 2.5);
    private final PathProvider practiceProvider = new TimedPathProvider("practice", 2.0);

    private final CommandAction score = ExampleCommands.command("score-preload");
    private final CommandAction intake = ExampleCommands.command("intake-marker");
    private final CommandAction balance = ExampleCommands.command("balance-marker");

    public final AutoRuntime runtime = Autos.runtime(
            Autos.path("Practice timed path", practiceProvider, "leave-community",
                    Autos.marker("intake", intake),
                    Autos.marker("balance", balance)),
            Autos.path("PathPlanner auto", pathPlanner, "PathPlannerLeaveAndScore",
                    Autos.marker("score", score)),
            Autos.path("Choreo trajectory", choreoFactoryStyle, "ChoreoLeaveAndScore"),
            Autos.routine("Inline command routine", ExampleCommands.command("inline-two-note")));

    public final PathGraph markers = PathGraph.of(runtime.selectedRoutine());

    public void loadProviderExamples() {
        PathAction pathPlannerAction = pathPlanner.path("PathPlannerLeaveAndScore").seconds(3.0);
        PathAction choreoFileAction = choreoFiles.path("ChoreoLeaveAndScore").seconds(2.5);
        PathAction choreoFactoryAction = choreoFactoryStyle.path("ChoreoSplitA").seconds(1.25);
        PathAction handAuthoredAction = Paths.of("practice", "HandAuthoredTaxi").seconds(2.0);

        pathPlannerAction.name();
        choreoFileAction.name();
        choreoFactoryAction.name();
        handAuthoredAction.name();
    }
}
