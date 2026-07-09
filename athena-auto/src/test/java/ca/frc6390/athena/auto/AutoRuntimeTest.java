package ca.frc6390.athena.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.commands.CommandState;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import ca.frc6390.athena.mechanism.core.PathRuntime;
import ca.frc6390.athena.mechanism.core.PathState;
import ca.frc6390.athena.mechanism.core.Paths;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class AutoRuntimeTest {
    @Test
    void indexesNormalizedRoutineNamesAndRejectsDuplicates() {
        CommandState state = CommandState.create("score").build();

        AutoRuntime runtime = Autos.runtime(Autos.routine(" score ", state), Autos.routine("leave", state));

        assertEquals(List.of("score", "leave"), List.copyOf(runtime.routineNames()));
        assertTrue(runtime.find("score").isPresent());
        assertEquals("score", runtime.selectedName());
        assertThrows(IllegalArgumentException.class,
                () -> Autos.runtime(Autos.routine("score", state), Autos.routine(" score ", state)));
    }

    @Test
    void providerBackedRoutineLoadsSelectedStateLazilyAndResetsIt() {
        FakeProvider provider = new FakeProvider();
        AutoRoutine routine = Autos.path("drive", provider, " taxi ");
        AutoRuntime runtime = Autos.runtime(routine);

        CommandState first = runtime.selectedState();
        CommandState second = runtime.selectedState();
        runtime.reset();
        CommandState third = runtime.selectedState();

        assertSame(first, second);
        assertEquals("pathplanner:taxi", first.name());
        assertEquals("pathplanner:taxi", third.name());
        assertEquals(2, provider.loads.get());
        assertEquals(Paths.pathPlanner("taxi"), provider.path(" taxi "));
    }

    @Test
    void markerBindingsAreValidatedByRoutine() {
        CommandState markerState = CommandState.create("shoot").build();

        AutoRoutine routine = Autos.routine(
                "score",
                CommandState.create("score").build(),
                Autos.marker(" shoot ", markerState),
                Autos.marker("intake", markerState));

        assertEquals(List.of("shoot", "intake"), routine.markers().stream().map(PathMarkerBinding::marker).toList());
        assertThrows(IllegalArgumentException.class,
                () -> Autos.routine(
                        "bad",
                        CommandState.create("bad").build(),
                        Autos.marker("same", markerState),
                        Autos.marker(" same ", markerState)));
    }

    @Test
    void pathGraphIndexesAndDispatchesMarkerCommands() {
        AtomicInteger initialize = new AtomicInteger();
        AtomicInteger execute = new AtomicInteger();
        AtomicInteger end = new AtomicInteger();
        AtomicInteger finishedChecks = new AtomicInteger();
        CommandState markerState = CommandState.create("shoot")
                .onInitialize(initialize::incrementAndGet)
                .onExecute(execute::incrementAndGet)
                .until(() -> finishedChecks.incrementAndGet() >= 2)
                .onEnd(end::incrementAndGet)
                .build();
        PathGraph graph = PathGraph.of(Autos.routine(
                "score",
                CommandState.create("score").build(),
                Autos.marker(" shoot ", markerState)));

        assertEquals(List.of("shoot"), List.copyOf(graph.markerNames()));
        assertSame(markerState, graph.marker("shoot").orElseThrow());
        assertEquals(false, graph.trigger(" shoot "));
        assertEquals(true, graph.trigger("shoot"));
        assertEquals(true, graph.trigger("shoot"));

        assertEquals(2, initialize.get());
        assertEquals(3, execute.get());
        assertEquals(2, end.get());
    }

    @Test
    void pathGraphRejectsDuplicateMarkersAcrossRoutinesAndEndsActiveMarkers() {
        AtomicInteger end = new AtomicInteger();
        CommandState markerState = CommandState.create("hold")
                .onEnd(end::incrementAndGet)
                .build();

        PathGraph graph = PathGraph.of(Autos.routine(
                "one",
                CommandState.create("one").build(),
                Autos.marker("hold", markerState)));

        graph.trigger("hold");
        graph.endAll(true);

        assertEquals(1, end.get());
        assertThrows(IllegalArgumentException.class,
                () -> PathGraph.of(
                        Autos.routine("one", CommandState.create("one").build(), Autos.marker("same", markerState)),
                        Autos.routine("two", CommandState.create("two").build(), Autos.marker(" same ", markerState))));
    }

    @Test
    void ownsSelectedCommandLifecycle() {
        AtomicInteger initialize = new AtomicInteger();
        AtomicInteger execute = new AtomicInteger();
        AtomicInteger end = new AtomicInteger();
        AtomicInteger finishedChecks = new AtomicInteger();
        CommandState state = CommandState.create("two-cycle")
                .onInitialize(initialize::incrementAndGet)
                .onExecute(execute::incrementAndGet)
                .until(() -> finishedChecks.incrementAndGet() >= 2)
                .onEnd(end::incrementAndGet)
                .build();
        AutoRuntime runtime = Autos.runtime(Autos.routine("auto", state));

        assertEquals(false, runtime.periodic());
        assertEquals(true, runtime.periodic());
        runtime.execute();
        runtime.end(true);

        assertEquals(2, initialize.get());
        assertEquals(3, execute.get());
        assertEquals(2, end.get());
    }

    private static final class FakeProvider implements PathProvider {
        private final AtomicInteger loads = new AtomicInteger();

        @Override
        public PathState path(String pathName) {
            return Paths.pathPlanner(pathName);
        }

        @Override
        public CommandState load(String pathName) {
            loads.incrementAndGet();
            return CommandState.create(path(pathName).key()).build();
        }

        @Override
        public PathRuntime runtime() {
            return new PathRuntime() {
                @Override
                public boolean isFinished(PathState path, MechanismContext context) {
                    return true;
                }
            };
        }
    }
}
