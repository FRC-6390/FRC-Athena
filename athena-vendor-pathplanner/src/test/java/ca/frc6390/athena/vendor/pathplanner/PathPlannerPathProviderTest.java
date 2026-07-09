package ca.frc6390.athena.vendor.pathplanner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.auto.PathGraph;
import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.PathAction;
import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.Test;

class PathPlannerPathProviderTest {
    @Test
    void pathNamesAreLoadedOnceAndReturnedAsClientProvided() {
        FakeClient client = new FakeClient(List.of("One", "Two"));
        PathPlannerPathProvider provider = new PathPlannerPathProvider(client);

        assertEquals(List.of("One", "Two"), provider.pathNames());
        assertEquals(List.of("One", "Two"), provider.pathNames());

        assertEquals(1, client.autoNameCalls);
    }

    @Test
    void pathNamesAreNormalizedAndCached() {
        PathPlannerPathProvider provider = new PathPlannerPathProvider(new FakeClient(List.of()));

        PathAction first = provider.path("  Blue Auto  ");
        PathAction second = provider.path("Blue Auto");
        PathAction defaultPath = provider.path(" ");

        assertSame(first, second);
        assertEquals("pathplanner", first.source());
        assertEquals("Blue Auto", first.name());
        assertEquals("pathplanner:Blue Auto", first.key());
        assertEquals("default", defaultPath.name());
    }

    @Test
    void loadWrapsCommandLifecycle() {
        FakeClient client = new FakeClient(List.of());
        PathPlannerPathProvider provider = new PathPlannerPathProvider(client);

        CommandAction Action = provider.load("  Auto A  ");
        Action.onInitialize().run();
        Action.onExecute().run();
        assertFalse(Action.isFinished().getAsBoolean());
        client.lastCommand.finished = true;
        assertEquals(true, Action.isFinished().getAsBoolean());
        Action.onEnd().run();

        assertEquals("pathplanner:Auto A", Action.name());
        assertEquals(List.of("Auto A"), client.buildAutoNames);
        assertEquals(List.of("initialize", "execute", "isFinished", "isFinished", "end:false"),
                client.lastCommand.events);
    }

    @Test
    void missingPathFailuresArePropagatedFromClient() {
        FakeClient client = new FakeClient(List.of());
        client.missingAutoName = "Missing";
        PathPlannerPathProvider provider = new PathPlannerPathProvider(client);

        IllegalArgumentException loadFailure = assertThrows(IllegalArgumentException.class,
                () -> provider.load(" Missing "));
        IllegalArgumentException commandFailure = assertThrows(IllegalArgumentException.class,
                () -> provider.command("Missing"));

        assertEquals("Missing PathPlanner auto 'Missing'.", loadFailure.getMessage());
        assertEquals("Missing PathPlanner auto 'Missing'.", commandFailure.getMessage());
        assertEquals(List.of("Missing", "Missing"), client.buildAutoNames);
    }

    @Test
    void providerBackedRoutineKeepsMarkerBindingsForPathGraph() {
        FakeClient client = new FakeClient(List.of());
        PathPlannerPathProvider provider = new PathPlannerPathProvider(client);
        FakeCommand markerCommand = new FakeCommand();
        markerCommand.finished = true;
        CommandAction markerState = CommandAction.create("shoot")
                .onInitialize(markerCommand::initialize)
                .onExecute(markerCommand::execute)
                .until(markerCommand::isFinished)
                .onEnd(() -> markerCommand.end(false))
                .build();

        AutoRoutine routine = Autos.path("score", provider, "Main", Autos.marker(" shoot ", markerState));
        PathGraph graph = PathGraph.of(routine);

        assertEquals(List.of("shoot"), List.copyOf(routine.markers().stream().map(binding -> binding.marker()).toList()));
        assertSame(markerState, graph.marker("shoot").orElseThrow());
        assertTrue(graph.trigger(" shoot "));
        assertEquals(List.of("initialize", "execute", "isFinished", "end:false"), markerCommand.events);
    }

    @Test
    void runtimeCachesActiveCommandUntilEnded() {
        FakeClient client = new FakeClient(List.of());
        PathPlannerPathProvider provider = new PathPlannerPathProvider(client);
        PathRuntime runtime = provider.runtime();
        PathAction path = provider.path("Drive Out");

        runtime.initialize(path, null);
        runtime.execute(path, null);
        runtime.isFinished(path, null);
        runtime.end(path, null, true);

        assertEquals(List.of("Drive Out"), client.buildAutoNames);
        assertEquals(List.of("initialize", "execute", "isFinished", "end:true"), client.commands.get(0).events);
    }

    private static final class FakeClient implements PathPlannerPathProvider.PathPlannerClient {
        private final List<String> autoNames;
        private final List<String> buildAutoNames = new ArrayList<>();
        private final List<FakeCommand> commands = new ArrayList<>();
        private int autoNameCalls;
        private FakeCommand lastCommand;
        private String missingAutoName;

        private FakeClient(List<String> autoNames) {
            this.autoNames = autoNames;
        }

        @Override
        public Command buildAuto(String autoName) {
            buildAutoNames.add(autoName);
            if (autoName.equals(missingAutoName)) {
                throw new IllegalArgumentException("Missing PathPlanner auto '" + autoName + "'.");
            }
            lastCommand = new FakeCommand();
            commands.add(lastCommand);
            return lastCommand;
        }

        @Override
        public List<String> autoNames() {
            autoNameCalls++;
            return autoNames;
        }
    }

    private static final class FakeCommand extends Command {
        private final List<String> events = new ArrayList<>();
        private boolean finished;

        @Override
        public void initialize() {
            events.add("initialize");
        }

        @Override
        public void execute() {
            events.add("execute");
        }

        @Override
        public void end(boolean interrupted) {
            events.add("end:" + interrupted);
        }

        @Override
        public boolean isFinished() {
            events.add("isFinished");
            return finished;
        }
    }
}
