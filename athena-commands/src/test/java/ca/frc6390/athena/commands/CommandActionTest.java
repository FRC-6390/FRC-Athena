package ca.frc6390.athena.commands;

import static org.junit.jupiter.api.Assertions.assertDoesNotThrow;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class CommandActionTest {
    @Test
    void defaultsAreSafeNoOps() {
        CommandAction Action = CommandAction.create("  ").build();

        assertEquals("command", Action.name());
        assertFalse(Action.isFinished().getAsBoolean());
        assertDoesNotThrow(Action.onInitialize()::run);
        assertDoesNotThrow(Action.onExecute()::run);
        assertDoesNotThrow(Action.onEnd()::run);
        assertTrue(Action.requirements().isEmpty());
    }

    @Test
    void requirementsAreTrimmedAndDeduplicated() {
        CommandAction Action = CommandAction.create("score")
                .requires(" arm ", "", null, "drive")
                .requires(List.of("arm", "intake"))
                .build();

        assertEquals(SetOrder.of("arm", "drive", "intake"), Action.requirements());
    }

    @Test
    void callbacksAndFinishConditionArePreserved() {
        AtomicInteger calls = new AtomicInteger();
        CommandAction Action = CommandAction.create("run")
                .onInitialize(calls::incrementAndGet)
                .onExecute(calls::incrementAndGet)
                .until(() -> true)
                .onEnd(calls::incrementAndGet)
                .build();

        Action.onInitialize().run();
        Action.onExecute().run();
        Action.onEnd().run();

        assertEquals(3, calls.get());
        assertTrue(Action.isFinished().getAsBoolean());
    }

    @Test
    void commandGraphOwnsLifecycleAndFinishedCommandsReleaseRequirements() {
        AtomicInteger initialize = new AtomicInteger();
        AtomicInteger execute = new AtomicInteger();
        AtomicInteger end = new AtomicInteger();
        AtomicInteger finishedChecks = new AtomicInteger();
        CommandAction Action = CommandAction.create("drive")
                .requires("drivetrain")
                .onInitialize(initialize::incrementAndGet)
                .onExecute(execute::incrementAndGet)
                .until(() -> finishedChecks.incrementAndGet() >= 2)
                .onEnd(end::incrementAndGet)
                .build();
        CommandGraph graph = new CommandGraph().schedule(Action);

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
        CommandAction first = CommandAction.create("first")
                .requires("arm")
                .onEnd(firstEnd::incrementAndGet)
                .build();
        CommandAction replacement = CommandAction.create("first")
                .requires("arm")
                .onEnd(replacementEnd::incrementAndGet)
                .build();
        CommandAction second = CommandAction.create("second")
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

    @Test
    void commandCanReplaceItselfByNameDuringExecution() {
        CommandGraph graph = new CommandGraph();
        AtomicInteger firstEnd = new AtomicInteger();
        AtomicInteger replacementExecute = new AtomicInteger();
        AtomicInteger replacementEnd = new AtomicInteger();
        CommandAction replacement = CommandAction.create("drive")
                .requires("drivetrain")
                .onExecute(replacementExecute::incrementAndGet)
                .until(() -> true)
                .onEnd(replacementEnd::incrementAndGet)
                .build();
        CommandAction first = CommandAction.create("drive")
                .requires("drivetrain")
                .onExecute(() -> graph.schedule(replacement))
                .until(() -> true)
                .onEnd(firstEnd::incrementAndGet)
                .build();

        graph.schedule(first);
        assertEquals(SetOrder.of("drive"), new java.util.LinkedHashSet<>(graph.periodic()));
        assertEquals(SetOrder.of("drive"), new java.util.LinkedHashSet<>(graph.activeCommandNames()));
        assertEquals(1, firstEnd.get());
        assertEquals(0, replacementExecute.get());

        assertEquals(SetOrder.of("drive"), new java.util.LinkedHashSet<>(graph.periodic()));
        assertTrue(graph.activeCommandNames().isEmpty());
        assertEquals(1, replacementExecute.get());
        assertEquals(1, replacementEnd.get());
    }

    @Test
    void commandScheduledFromConflictEndOwnsRequirementWithoutGhostOwner() {
        CommandGraph graph = new CommandGraph();
        AtomicInteger displacedExecute = new AtomicInteger();
        AtomicInteger requestedExecute = new AtomicInteger();
        CommandAction displaced = CommandAction.create("displaced")
                .requires("arm")
                .onExecute(displacedExecute::incrementAndGet)
                .build();
        CommandAction requested = CommandAction.create("requested")
                .requires("arm")
                .onExecute(requestedExecute::incrementAndGet)
                .build();
        CommandAction original = CommandAction.create("original")
                .requires("arm")
                .onEnd(() -> graph.schedule(displaced))
                .build();

        graph.schedule(original).schedule(requested);
        graph.periodic();

        // The callback request is ordered after the outer request, so it wins. Before mutation
        // serialization both commands remained active while only one appeared to own "arm".
        assertEquals(SetOrder.of("displaced"), new java.util.LinkedHashSet<>(graph.activeCommandNames()));
        assertEquals(1, displacedExecute.get());
        assertEquals(0, requestedExecute.get());
    }

    @Test
    void lifecycleCallbacksMayMutateGraphAcrossEveryTransition() {
        CommandGraph graph = new CommandGraph();
        AtomicInteger finalExecutions = new AtomicInteger();
        CommandAction terminal = CommandAction.create("terminal")
                .requires("arm")
                .onExecute(finalExecutions::incrementAndGet)
                .build();
        CommandAction fromEnd = CommandAction.create("fromEnd")
                .requires("arm")
                .onInitialize(() -> graph.schedule(terminal))
                .build();
        CommandAction fromExecute = CommandAction.create("fromExecute")
                .requires("arm")
                .onExecute(() -> graph.cancel("fromExecute"))
                .onEnd(() -> graph.schedule(fromEnd))
                .build();

        graph.schedule(fromExecute);
        assertDoesNotThrow(graph::periodic);
        assertEquals(SetOrder.of("terminal"), new java.util.LinkedHashSet<>(graph.activeCommandNames()));

        graph.periodic();
        assertEquals(1, finalExecutions.get());
    }

    @Test
    void cancelAllIsSafeWhenEndCallbacksScheduleCommands() {
        CommandGraph graph = new CommandGraph();
        CommandAction afterCancel = CommandAction.create("afterCancel").requires("arm").build();
        graph.schedule(CommandAction.create("first")
                .requires("arm")
                .onEnd(() -> graph.schedule(afterCancel))
                .build());

        assertDoesNotThrow(graph::cancelAll);

        assertEquals(SetOrder.of("afterCancel"), new java.util.LinkedHashSet<>(graph.activeCommandNames()));
    }

    @Test
    void activeCommandNamesIsAnImmutableSnapshot() {
        AtomicInteger end = new AtomicInteger();
        CommandGraph graph = new CommandGraph().schedule(CommandAction.create("drive")
                .requires("drivetrain")
                .onEnd(end::incrementAndGet)
                .build());
        var names = graph.activeCommandNames();

        assertThrows(UnsupportedOperationException.class, names::clear);
        assertEquals(SetOrder.of("drive"), new java.util.LinkedHashSet<>(graph.activeCommandNames()));
        assertTrue(graph.cancel("drive"));
        assertEquals(1, end.get());
        assertEquals(SetOrder.of("drive"), new java.util.LinkedHashSet<>(names));
    }

    private static final class SetOrder {
        static java.util.Set<String> of(String... values) {
            return new java.util.LinkedHashSet<>(java.util.List.of(values));
        }
    }
}
