package frc.robot;

import ca.frc6390.athena.auto.AutoRuntime;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.auto.PathGraph;
import ca.frc6390.athena.auto.PathProvider;
import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.mechanism.core.Events;
import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.PathAction;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.Paths;
import ca.frc6390.athena.vendor.choreo.ChoreoPathProvider;
import ca.frc6390.athena.vendor.pathplanner.PathPlannerPathProvider;
import ca.frc6390.athena.wpilib.lifecycle.AthenaRobot;

public final class Robot extends AthenaRobot {
    private final PathPlannerPathProvider pathPlanner = new PathPlannerPathProvider();
    private final ChoreoPathProvider choreoFiles = new ChoreoPathProvider();
    private final PathProvider choreoFactoryStyle = new TimedPathProvider("choreo", 2.5);
    private final PathProvider practiceProvider = new TimedPathProvider("practice", 2.0);

    private final CommandAction score = command("score-preload");
    private final CommandAction intake = command("intake-marker");
    private final CommandAction balance = command("balance-marker");

    private final AutoRuntime autos = Autos.runtime(
            Autos.path("Practice timed path", practiceProvider, "leave-community",
                    Autos.marker("intake", intake),
                    Autos.marker("balance", balance)),
            Autos.path("PathPlanner auto", pathPlanner, "PathPlannerLeaveAndScore",
                    Autos.marker("score", score)),
            Autos.path("Choreo trajectory", choreoFactoryStyle, "ChoreoLeaveAndScore"),
            Autos.routine("Inline command routine", command("inline-two-note")));

    private final PathGraph markers = PathGraph.of(autos.selectedRoutine());

    @SuppressWarnings("unused")
    public final HookBinding selectDefault = Events.robotInit().onStart(() -> autos.select("Practice timed path"));

    @Override
    protected void configure() {
        athena().auto(autos, markers);

        PathAction pathPlannerAction = pathPlanner.path("PathPlannerLeaveAndScore").seconds(3.0);
        PathAction choreoFileAction = choreoFiles.path("ChoreoLeaveAndScore").seconds(2.5);
        PathAction choreoFactoryAction = choreoFactoryStyle.path("ChoreoSplitA").seconds(1.25);
        PathAction handAuthoredAction = Paths.of("practice", "HandAuthoredTaxi").seconds(2.0);

        pathPlannerAction.name();
        choreoFileAction.name();
        choreoFactoryAction.name();
        handAuthoredAction.name();
    }

    private static CommandAction command(String name) {
        return CommandAction.create(name)
                .onInitialize(() -> {})
                .onExecute(() -> {})
                .until(() -> false)
                .onEnd(() -> {})
                .build();
    }

    private static final class TimedPathProvider implements PathProvider {
        private final String source;
        private final double seconds;

        private TimedPathProvider(String source, double seconds) {
            this.source = source;
            this.seconds = seconds;
        }

        @Override
        public PathAction path(String pathName) {
            return Paths.of(source, pathName).seconds(seconds);
        }

        @Override
        public CommandAction load(String pathName) {
            return CommandAction.create(source + ":" + pathName)
                    .onExecute(() -> {})
                    .until(() -> false)
                    .build();
        }

        @Override
        public PathRuntime runtime() {
            return new PathRuntime() {
                @Override
                public boolean isFinished(PathAction path, MechanismContext context) {
                    return context.timeInStateSeconds() >= path.expectedDurationSeconds().orElse(seconds);
                }
            };
        }
    }

}
