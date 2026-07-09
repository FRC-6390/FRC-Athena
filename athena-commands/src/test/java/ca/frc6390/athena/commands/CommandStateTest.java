package ca.frc6390.athena.commands;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class CommandStateTest {
    @Test
    void defaultsAreSafeNoOps() {
        CommandState state = CommandState.create("  ").build();

        assertEquals("command", state.name());
        assertFalse(state.isFinished().getAsBoolean());
        assertDoesNotThrow(state.onInitialize()::run);
        assertDoesNotThrow(state.onExecute()::run);
        assertDoesNotThrow(state.onEnd()::run);
        assertTrue(state.requirements().isEmpty());
    }

    @Test
    void requirementsAreTrimmedAndDeduplicated() {
        CommandState state = CommandState.create("score")
                .requires(" arm ", "", null, "drive")
                .requires(List.of("arm", "intake"))
                .build();

        assertEquals(SetOrder.of("arm", "drive", "intake"), state.requirements());
    }

    @Test
    void callbacksAndFinishConditionArePreserved() {
        AtomicInteger calls = new AtomicInteger();
        CommandState state = CommandState.create("run")
                .onInitialize(calls::incrementAndGet)
                .onExecute(calls::incrementAndGet)
                .until(() -> true)
                .onEnd(calls::incrementAndGet)
                .build();

        state.onInitialize().run();
        state.onExecute().run();
        state.onEnd().run();

        assertEquals(3, calls.get());
        assertTrue(state.isFinished().getAsBoolean());
    }

    @Test
    void commandGraphOwnsLifecycleAndFinishedCommandsReleaseRequirements() {
        AtomicInteger initialize = new AtomicInteger();
        AtomicInteger execute = new AtomicInteger();
        AtomicInteger end = new AtomicInteger();
        AtomicInteger finishedChecks = new AtomicInteger();
        CommandState state = CommandState.create("drive")
                .requires("drivetrain")
                .onInitialize(initialize::incrementAndGet)
                .onExecute(execute::incrementAndGet)
                .until(() -> finishedChecks.incrementAndGet() >= 2)
                .onEnd(end::incrementAndGet)
                .build();
        CommandGraph graph = new CommandGraph().schedule(state);

        assertEquals(SetOrder.of("drive"), new java.util.LinkedHashSet<>(graph.activeCommandNames()));
        assertTrue(graph.periodic().isEmpty());
        assertEquals(SetOrder.of("drive"), new java.util.LinkedHashSet<>(graph.periodic()));

        assertTrue(graph.activeCommandNames().isEmpty());
        assertEquals(1, initialize.get());
        assertEquals(2, execute.get());
        assertEquals(1, end.get());
    }

    @Test
    void commandGraphCancelsRequirementConflictsAndReplacements() {
        AtomicInteger firstEnd = new AtomicInteger();
        AtomicInteger replacementEnd = new AtomicInteger();
        AtomicInteger secondEnd = new AtomicInteger();
        CommandState first = CommandState.create("first")
                .requires("arm")
                .onEnd(firstEnd::incrementAndGet)
                .build();
        CommandState replacement = CommandState.create("first")
                .requires("arm")
                .onEnd(replacementEnd::incrementAndGet)
                .build();
        CommandState second = CommandState.create("second")
                .requires("arm")
                .onEnd(secondEnd::incrementAndGet)
                .build();
        CommandGraph graph = new CommandGraph();

        graph.schedule(first).schedule(replacement).schedule(second);

        assertEquals(1, firstEnd.get());
        assertEquals(1, replacementEnd.get());
        assertEquals(0, secondEnd.get());
        assertEquals(SetOrder.of("second"), new java.util.LinkedHashSet<>(graph.activeCommandNames()));
        assertTrue(graph.cancel(" second "));
        assertEquals(1, secondEnd.get());
        assertTrue(graph.activeCommandNames().isEmpty());
    }

    private static final class SetOrder {
        static java.util.Set<String> of(String... values) {
            return new java.util.LinkedHashSet<>(java.util.List.of(values));
        }
    }
}
