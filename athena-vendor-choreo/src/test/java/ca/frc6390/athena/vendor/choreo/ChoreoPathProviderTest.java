package ca.frc6390.athena.vendor.choreo;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import ca.frc6390.athena.auto.AutoRoutine;
import ca.frc6390.athena.auto.PathGraph;
import ca.frc6390.athena.auto.PathMarkerBinding;
import ca.frc6390.athena.commands.CommandAction;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.PathAction;
import choreo.trajectory.EventMarker;
import choreo.trajectory.Trajectory;
import choreo.trajectory.TrajectorySample;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.Test;

class ChoreoPathProviderTest {
    @Test
    void providerNormalizesAndCachesPathsAndTrajectoryLookups() {
        FakeChoreoClient client = new FakeChoreoClient(List.of("Score", "Leave"));
        ChoreoPathProvider provider = new ChoreoPathProvider(client);

        PathAction first = provider.path("  Score  ");
        PathAction second = provider.path("Score");

        assertSame(first, second);
        assertEquals("choreo", first.source());
        assertEquals("Score", first.name());
        assertEquals(Optional.empty(), provider.trajectory("  Score  "));
        assertEquals(Optional.empty(), provider.trajectory("Score"));
        assertEquals(List.of(), provider.markerNames("Score"));
        assertEquals(List.of("Score"), client.loadedTrajectories);
    }

    @Test
    void providerCachesDiscoveredNames() {
        FakeChoreoClient client = new FakeChoreoClient(List.of("One", "Two"));
        ChoreoPathProvider provider = new ChoreoPathProvider(client);

        assertEquals(List.of("One", "Two"), provider.pathNames());
        assertEquals(List.of("One", "Two"), provider.pathNames());

        assertEquals(1, client.nameCalls);
    }

    @Test
    void providerConvertsTrajectoryMarkersToPathGraphBindings() {
        FakeCommand shootCommand = new FakeCommand();
        shootCommand.finished = true;
        FakeCommand intakeCommand = new FakeCommand();
        intakeCommand.finished = true;
        CommandAction shootState = commandState("shoot", shootCommand);
        CommandAction intakeState = commandState("intake", intakeCommand);
        FakeChoreoClient client = new FakeChoreoClient(
                List.of("Score"),
                Map.of("Score", trajectory("Score",
                        new EventMarker(0.5, " shoot "),
                        new EventMarker(0.8, "intake"),
                        new EventMarker(1.0, "shoot"))));
        ChoreoPathProvider provider = new ChoreoPathProvider(client);

        List<PathMarkerBinding> bindings = provider.markerBindings(
                " Score ",
                orderedCommands(Map.entry(" shoot ", shootState), Map.entry("intake", intakeState)));
        PathGraph graph = PathGraph.of(new AutoRoutine(
                "score",
                () -> CommandAction.create("score").build(),
                bindings));

        assertEquals(List.of("shoot", "intake"), provider.markerNames("Score"));
        assertEquals(List.of("shoot", "intake"), bindings.stream().map(PathMarkerBinding::marker).toList());
        assertTrue(graph.trigger(" shoot "));
        assertTrue(graph.trigger("intake"));
        assertEquals(List.of("initialize", "execute", "isFinished", "end:false"), shootCommand.events);
        assertEquals(List.of("initialize", "execute", "isFinished", "end:false"), intakeCommand.events);
    }

    @Test
    void providerRejectsUnboundTrajectoryMarkerCommands() {
        FakeChoreoClient client = new FakeChoreoClient(
                List.of("Score"),
                Map.of("Score", trajectory("Score", new EventMarker(0.5, "shoot"))));
        ChoreoPathProvider provider = new ChoreoPathProvider(client);

        IllegalArgumentException failure = assertThrows(IllegalArgumentException.class,
                () -> provider.markerBindings("Score", Map.of()));

        assertEquals("Missing command for Choreo marker 'shoot'.", failure.getMessage());
    }

    @Test
    void adapterDelegatesNamedCommandsAndNormalizesNames() {
        FakeFactoryClient client = new FakeFactoryClient();
        ChoreoPathAdapter adapter = new ChoreoPathAdapter(client);

        assertSame(client.trajectoryCommand, adapter.trajectoryCommand("  Main  "));
        assertSame(client.splitTrajectoryCommand, adapter.trajectoryCommand("  Main  ", 2));
        assertSame(client.resetCommand, adapter.resetOdometryCommand("  Main  "));
        assertSame(client.splitResetCommand, adapter.resetOdometryCommand("  Main  ", 3));
        assertSame(client.routineCommand, adapter.routineCommand("  Routine  "));
        assertSame(client.warmupCommand, adapter.warmupCommand());

        assertEquals(List.of(
                "trajectory:Main",
                "trajectorySplit:Main:2",
                "reset:Main",
                "resetSplit:Main:3",
                "routine:Routine",
                "warmup"), client.calls);
    }

    @Test
    void adapterLoadWrapsTrajectoryCommandLifecycle() {
        FakeFactoryClient client = new FakeFactoryClient();
        ChoreoPathAdapter adapter = new ChoreoPathAdapter(client);

        CommandAction Action = adapter.load("  Main  ");
        Action.onInitialize().run();
        Action.onExecute().run();
        assertFalse(Action.isFinished().getAsBoolean());
        client.trajectoryCommand.finished = true;
        assertEquals(true, Action.isFinished().getAsBoolean());
        Action.onEnd().run();

        assertEquals("choreo:Main", Action.name());
        assertEquals(List.of("trajectory:Main"), client.calls);
        assertEquals(List.of("initialize", "execute", "isFinished", "isFinished", "end:false"),
                client.trajectoryCommand.events);
    }

    @Test
    void adapterRuntimesCacheActiveCommandUntilEnded() {
        FakeFactoryClient client = new FakeFactoryClient();
        ChoreoPathAdapter adapter = new ChoreoPathAdapter(client);
        PathRuntime trajectoryRuntime = adapter.trajectoryRuntime();
        PathRuntime routineRuntime = adapter.routineRuntime();
        PathAction path = adapter.path("Main");

        trajectoryRuntime.initialize(path, null);
        trajectoryRuntime.execute(path, null);
        trajectoryRuntime.end(path, null, true);
        routineRuntime.initialize(path, null);
        routineRuntime.execute(path, null);
        routineRuntime.end(path, null, false);

        assertEquals(List.of("trajectory:Main", "routine:Main"), client.calls);
        assertEquals(List.of("initialize", "execute", "end:true"), client.trajectoryCommand.events);
        assertEquals(List.of("initialize", "execute", "end:false"), client.routineCommand.events);
    }

    private static CommandAction commandState(String name, FakeCommand command) {
        return CommandAction.create(name)
                .onInitialize(command::initialize)
                .onExecute(command::execute)
                .until(command::isFinished)
                .onEnd(() -> command.end(false))
                .build();
    }

    @SafeVarargs
    private static Map<String, CommandAction> orderedCommands(Map.Entry<String, CommandAction>... entries) {
        Map<String, CommandAction> commands = new LinkedHashMap<>();
        for (Map.Entry<String, CommandAction> entry : entries) {
            commands.put(entry.getKey(), entry.getValue());
        }
        return commands;
    }

    private static Trajectory<TestSample> trajectory(String name, EventMarker... markers) {
        return new Trajectory<TestSample>(name, List.<TestSample>of(), List.of(), List.of(markers));
    }

    private static final class FakeChoreoClient implements ChoreoPathProvider.ChoreoClient {
        private final List<String> names;
        private final Map<String, Trajectory<? extends TrajectorySample<?>>> trajectories;
        private final List<String> loadedTrajectories = new ArrayList<>();
        private int nameCalls;

        private FakeChoreoClient(List<String> names) {
            this(names, Map.of());
        }

        private FakeChoreoClient(List<String> names, Map<String, Trajectory<? extends TrajectorySample<?>>> trajectories) {
            this.names = names;
            this.trajectories = trajectories;
        }

        @Override
        public Optional<Trajectory<? extends TrajectorySample<?>>> loadTrajectory(String trajectoryName) {
            loadedTrajectories.add(trajectoryName);
            return Optional.ofNullable(trajectories.get(trajectoryName));
        }

        @Override
        public List<String> trajectoryNames() {
            nameCalls++;
            return names;
        }
    }

    private static final class TestSample implements TrajectorySample<TestSample> {
        @Override
        public double getTimestamp() {
            return 0.0;
        }

        @Override
        public Pose2d getPose() {
            return new Pose2d();
        }

        @Override
        public ChassisSpeeds getChassisSpeeds() {
            return new ChassisSpeeds();
        }

        @Override
        public TestSample interpolate(TestSample endValue, double t) {
            return this;
        }

        @Override
        public TestSample flipped() {
            return this;
        }

        @Override
        public TestSample mirrorX() {
            return this;
        }

        @Override
        public TestSample mirrorY() {
            return this;
        }

        @Override
        public TestSample rotateAround() {
            return this;
        }

        @Override
        public TestSample offsetBy(double timestampOffset) {
            return this;
        }
    }

    private static final class FakeFactoryClient implements ChoreoPathAdapter.FactoryClient {
        private final List<String> calls = new ArrayList<>();
        private final FakeCommand trajectoryCommand = new FakeCommand();
        private final FakeCommand splitTrajectoryCommand = new FakeCommand();
        private final FakeCommand resetCommand = new FakeCommand();
        private final FakeCommand splitResetCommand = new FakeCommand();
        private final FakeCommand routineCommand = new FakeCommand();
        private final FakeCommand warmupCommand = new FakeCommand();

        @Override
        public Command trajectoryCommand(String trajectoryName) {
            calls.add("trajectory:" + trajectoryName);
            return trajectoryCommand;
        }

        @Override
        public Command trajectoryCommand(String trajectoryName, int splitIndex) {
            calls.add("trajectorySplit:" + trajectoryName + ":" + splitIndex);
            return splitTrajectoryCommand;
        }

        @Override
        public Command resetOdometryCommand(String trajectoryName) {
            calls.add("reset:" + trajectoryName);
            return resetCommand;
        }

        @Override
        public Command resetOdometryCommand(String trajectoryName, int splitIndex) {
            calls.add("resetSplit:" + trajectoryName + ":" + splitIndex);
            return splitResetCommand;
        }

        @Override
        public Command routineCommand(String routineName) {
            calls.add("routine:" + routineName);
            return routineCommand;
        }

        @Override
        public Command warmupCommand() {
            calls.add("warmup");
            return warmupCommand;
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
