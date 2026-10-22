package ca.frc6390.athena.vendor.pathplanner;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;

import ca.frc6390.athena.auto.AutoChooser;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.PathAction;
import ca.frc6390.athena.mechanism.core.Paths;
import edu.wpi.first.wpilibj2.command.Command;
import org.junit.jupiter.api.Test;

class PathPlannerPathProviderTest {
    @Test
    void pathNamesAreLoadedOnceAndReturnedAsClientProvided() {
        FakeClient client = new FakeClient(List.of("One", "Two"));
        PathPlannerPathProvider provider = new PathPlannerPathProvider(client);

        provider.pathNames();
        await(provider);
        assertEquals(List.of("One", "Two"), provider.pathNames());
        assertEquals(List.of("One", "Two"), provider.pathNames());

        assertEquals(1, client.autoNameCalls);
    }

    @Test
    void declarativePathActionsAreCreatedWithoutTheProvider() {
        PathAction first = Paths.pathPlanner("  Blue Auto  ");
        PathAction defaultPath = Paths.pathPlanner(" ");

        assertEquals("pathplanner", first.source());
        assertEquals("Blue Auto", first.name());
        assertEquals("pathplanner:Blue Auto", first.key());
        assertEquals("default", defaultPath.name());
    }

    @Test
    void loadReturnsPathActionThatUsesTheStartupPreparedAuto() {
        FakeClient client = new FakeClient(List.of());
        PathPlannerPathProvider provider = new PathPlannerPathProvider(client);

        PathAction path = provider.load("  Auto A  ");
        PathRuntime runtime = provider.runtime();

        assertEquals("pathplanner", path.source());
        assertEquals("Auto A", path.name());
        assertEquals("pathplanner:Auto A", path.key());
        assertTrue(client.buildAutoNames.isEmpty(), "load must not build the auto");

        prepare(provider, path);
        runtime.initialize(path, null);
        runtime.execute(path, null);
        assertFalse(runtime.isFinished(path, null));
        client.lastCommand.finished = true;
        assertTrue(runtime.isFinished(path, null));
        runtime.end(path, null, false);

        assertEquals(List.of("Auto A"), client.buildAutoNames);
        assertEquals(List.of("initialize", "execute", "isFinished", "isFinished", "end:false"),
                client.lastCommand.events);
    }

    @Test
    void loadedPathIsAcceptedByAutoChooser() {
        PathPlannerPathProvider provider = new PathPlannerPathProvider(new FakeClient(List.of()));
        PathAction path = provider.load("Auto A");

        AutoChooser chooser = Autos.chooser("Autos").auto("Auto A", path);

        assertEquals(List.of("Auto A"), List.copyOf(chooser.optionNames()));
        assertSame(path, chooser.selectedAction());
    }

    @Test
    void missingPathFailuresArePropagatedWhenThePathStarts() {
        FakeClient client = new FakeClient(List.of());
        client.missingAutoName = "Missing";
        PathPlannerPathProvider provider = new PathPlannerPathProvider(client);
        PathAction path = provider.load(" Missing ");
        PathRuntime runtime = provider.runtime();

        IllegalStateException unprepared = assertThrows(IllegalStateException.class,
                () -> runtime.initialize(path, null));
        assertTrue(unprepared.getMessage().contains("not preloaded"));
        provider.preload(path);
        IllegalArgumentException startFailure = assertThrows(IllegalArgumentException.class,
                () -> runtime.initialize(path, null));
        IllegalArgumentException retryFailure = assertThrows(IllegalArgumentException.class,
                () -> runtime.initialize(path, null));
        IllegalArgumentException commandFailure = assertThrows(IllegalArgumentException.class,
                () -> provider.command("Missing"));

        assertEquals("Missing PathPlanner auto 'Missing'.", startFailure.getMessage());
        assertSame(startFailure, retryFailure, "failed initialization leaves no phantom active path");
        assertEquals("Missing PathPlanner auto 'Missing'.", commandFailure.getMessage());
        assertEquals(List.of("Missing", "Missing"), client.buildAutoNames);
    }

    @Test
    void runtimeCachesActiveCommandUntilEnded() {
        FakeClient client = new FakeClient(List.of());
        PathPlannerPathProvider provider = new PathPlannerPathProvider(client);
        PathRuntime runtime = provider.runtime();
        PathAction path = Paths.pathPlanner("Drive Out");

        prepare(provider, path);
        runtime.initialize(path, null);
        runtime.execute(path, null);
        runtime.isFinished(path, null);
        runtime.end(path, null, true);

        assertEquals(List.of("Drive Out"), client.buildAutoNames);
        assertEquals(List.of("initialize", "execute", "isFinished", "end:true"), client.commands.get(0).events);
    }

    @Test
    void resetPoseOnlyUsesResetCommandWithoutBuildingTheAutoRuntime() {
        FakeClient client = new FakeClient(List.of("Start Here"));
        PathPlannerPathProvider provider = new PathPlannerPathProvider(client);
        PathRuntime runtime = provider.runtime();
        PathAction path = Paths.pathPlanner("Start Here").resetPoseOnly();

        prepare(provider, path);
        runtime.initialize(path, null);
        runtime.execute(path, null);
        runtime.end(path, null, false);

        assertEquals(List.of("Start Here"), client.resetPoseNames);
        assertTrue(client.buildAutoNames.isEmpty());
        assertEquals(List.of("initialize", "execute", "end:false"), client.commands.get(0).events);
    }

    @Test
    void unpreparedRuntimeNeverLoadsFilesAndPreparedCallbacksStayOnCaller() {
        Thread caller = Thread.currentThread();
        List<Thread> construction = new ArrayList<>();
        List<Thread> callbacks = new ArrayList<>();
        PathPlannerPathProvider.PathPlannerClient client = new PathPlannerPathProvider.PathPlannerClient() {
            @Override public Command buildAuto(String name) {
                construction.add(Thread.currentThread());
                return new FakeCommand() {
                    @Override public void initialize() { callbacks.add(Thread.currentThread()); }
                    @Override public void execute() { callbacks.add(Thread.currentThread()); }
                };
            }
            @Override public Command resetPose(String name) { return buildAuto(name); }
            @Override public List<String> autoNames() { return List.of("Prepared"); }
        };
        try (PathPlannerPathProvider provider = new PathPlannerPathProvider(client)) {
            PathAction path = provider.load("Prepared");
            PathRuntime runtime = provider.runtime();
            org.junit.jupiter.api.Assertions.assertTimeout(java.time.Duration.ofMillis(100), () -> {
                assertThrows(IllegalStateException.class, () -> runtime.initialize(path, null));
            });
            assertTrue(construction.isEmpty(), "an unprepared runtime must not construct commands or touch files");
            provider.prepareForStartup(path);
            assertEquals(List.of(caller), construction);
            runtime.initialize(path, null);
            runtime.execute(path, null);
            assertEquals(List.of(caller, caller), callbacks);
            runtime.end(path, null, false);
        }
    }

    @Test
    void repeatedRunsReusePreparedCommandAndEquivalentActionsHaveDistinctCommands() {
        FakeClient client = new FakeClient(List.of());
        try (PathPlannerPathProvider provider = new PathPlannerPathProvider(client)) {
            PathAction first = provider.load("Same");
            PathAction second = provider.load("Same");
            prepare(provider, first);
            prepare(provider, second);
            PathRuntime runtime = provider.runtime();
            assertSame(runtime, provider.runtime());
            runtime.initialize(first, null);
            runtime.initialize(second, null);
            runtime.end(first, null, false);
            runtime.end(second, null, false);
            prepare(provider, first);
            runtime.initialize(first, null);
            runtime.end(first, null, false);
            assertEquals(2, client.commands.size());
            assertEquals(List.of("initialize", "end:false", "initialize", "end:false"), client.commands.get(0).events);
            assertEquals(List.of("initialize", "end:false"), client.commands.get(1).events);
        }
    }

    @Test
    void initializationCallbackFailureIsCleanedUpAndCanBeRetried() {
        FakeCommand command = new FakeCommand() {
            @Override public void initialize() {
                super.initialize();
                throw new IllegalStateException("initialize failed");
            }
        };
        var client = new PathPlannerPathProvider.PathPlannerClient() {
            @Override public Command buildAuto(String name) { return command; }
            @Override public Command resetPose(String name) { return command; }
            @Override public List<String> autoNames() { return List.of(); }
        };
        try (PathPlannerPathProvider provider = new PathPlannerPathProvider(client)) {
            PathAction path = provider.load("Failed callback");
            prepare(provider, path);
            for (int retry = 0; retry < 2; retry++) {
                var failure = assertThrows(IllegalStateException.class, () -> provider.runtime().initialize(path, null));
                assertEquals("initialize failed", failure.getMessage());
            }
            assertEquals(List.of("initialize", "end:true", "initialize", "end:true"), command.events);
        }
    }

    @Test
    void startupFenceBuildsChooserCommandsAfterRobotInitConfiguration() {
        java.util.concurrent.atomic.AtomicBoolean configured = new java.util.concurrent.atomic.AtomicBoolean();
        java.util.concurrent.atomic.AtomicInteger builds = new java.util.concurrent.atomic.AtomicInteger();
        var client = new PathPlannerPathProvider.PathPlannerClient() {
            @Override public Command buildAuto(String name) {
                if (!configured.get()) throw new IllegalStateException("AutoBuilder is not configured");
                builds.incrementAndGet();
                return new FakeCommand();
            }
            @Override public Command resetPose(String name) { return buildAuto(name); }
            @Override public List<String> autoNames() { return List.of("Ordered"); }
        };
        PathPlannerPathProvider provider = new PathPlannerPathProvider(client);
        class Robot implements ca.frc6390.athena.mechanism.core.Mechanism {
            final PathPlannerPathProvider paths = provider;
            final ca.frc6390.athena.mechanism.core.HookBinding configure =
                    ca.frc6390.athena.mechanism.core.Events.robotInit().onStart(() -> configured.set(true));
            final AutoChooser autos = Autos.chooser().defaultAuto(provider.load("Ordered"));
        }
        try (var runtime = ca.frc6390.athena.robot.RobotRuntime.using(
                ca.frc6390.athena.hardware.runtime.HardwareGraph.using(
                        ca.frc6390.athena.hardware.backend.BackendRegistry.of())).register(new Robot())) {
            assertEquals(0, builds.get(), "registration cannot run before AutoBuilder's init hook");
            runtime.periodic(new ca.frc6390.athena.mechanism.core.MechanismContext(0, 0, 0.02, true, false, false),
                    new ca.frc6390.athena.mechanism.core.EventContext(0, 0.02,
                            ca.frc6390.athena.mechanism.core.LifecycleMode.ROBOT,
                            ca.frc6390.athena.mechanism.core.LifecyclePhase.INIT, true, false));
            assertTrue(configured.get());
            assertEquals(0, builds.get());
            runtime.preparePathsForStartup();
            runtime.preparePathsForStartup();
            assertEquals(1, builds.get(), "the post-init setup fence is idempotent");
            runtime.autoPeriodic(0.02, 0.02);
            assertThrows(IllegalStateException.class, runtime::preparePathsForStartup);
        }
    }

    private static void prepare(PathPlannerPathProvider provider, PathAction path) {
        provider.prepareForStartup(path);
        await(provider);
    }

    private static void await(PathPlannerPathProvider provider) {
        try { assertTrue(provider.awaitPrepared(java.time.Duration.ofSeconds(2))); }
        catch (InterruptedException interrupted) { Thread.currentThread().interrupt(); throw new AssertionError(interrupted); }
    }

    private static final class FakeClient implements PathPlannerPathProvider.PathPlannerClient {
        private final List<String> autoNames;
        private final List<String> buildAutoNames = new ArrayList<>();
        private final List<String> resetPoseNames = new ArrayList<>();
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
        public Command resetPose(String autoName) {
            resetPoseNames.add(autoName);
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

    private static class FakeCommand extends Command {
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
