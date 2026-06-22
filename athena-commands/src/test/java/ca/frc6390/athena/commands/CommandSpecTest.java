package ca.frc6390.athena.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;

import org.junit.jupiter.api.Test;

class CommandSpecTest {
    @Test
    void commandRunnerExecutesLifecycle() {
        AtomicInteger events = new AtomicInteger();
        AtomicInteger cycles = new AtomicInteger();

        CommandSpec spec = CommandSpec.create("intake")
                .onInitialize(events::incrementAndGet)
                .onExecute(() -> {
                    events.incrementAndGet();
                    cycles.incrementAndGet();
                })
                .until(() -> cycles.get() >= 2)
                .onEnd(events::incrementAndGet)
                .toSpec();

        CommandRunner runner = new CommandRunner(spec);

        assertFalse(runner.step());
        assertTrue(runner.step());
        assertTrue(runner.step());
        assertEquals(4, events.get());
    }

    @Test
    void sequenceRunsChildrenInOrderAcrossCycles() {
        List<String> events = new ArrayList<>();
        AtomicInteger firstCycles = new AtomicInteger();
        CommandSpec first = CommandSpec.create("first")
                .onExecute(() -> {
                    events.add("first");
                    firstCycles.incrementAndGet();
                })
                .until(() -> firstCycles.get() >= 2)
                .onEnd(() -> events.add("first.end"))
                .toSpec();
        CommandSpec second = CommandSpec.create("second")
                .onExecute(() -> events.add("second"))
                .until(() -> true)
                .onEnd(() -> events.add("second.end"))
                .toSpec();

        CommandRunner runner = new CommandRunner(CommandGroups.sequence("sequence", first, second));

        assertFalse(runner.step());
        assertFalse(runner.step());
        assertTrue(runner.step());
        assertEquals(List.of("first", "first", "first.end", "second", "second.end"), events);
    }

    @Test
    void commandsCarryNamedRequirements() {
        CommandSpec spec = CommandSpec.create("shoot")
                .requires("shooter", "indexer", "shooter")
                .toSpec();

        assertEquals(java.util.Set.of("shooter", "indexer"), spec.requirements());
    }

    @Test
    void parallelRunsChildrenUntilAllFinish() {
        List<String> events = new ArrayList<>();
        AtomicInteger slowCycles = new AtomicInteger();
        CommandSpec fast = CommandSpec.create("fast")
                .requires("drive")
                .onExecute(() -> events.add("fast"))
                .until(() -> true)
                .toSpec();
        CommandSpec slow = CommandSpec.create("slow")
                .requires("arm")
                .onExecute(() -> {
                    events.add("slow");
                    slowCycles.incrementAndGet();
                })
                .until(() -> slowCycles.get() >= 2)
                .toSpec();

        CommandRunner runner = new CommandRunner(CommandGroups.parallel("parallel", fast, slow));

        assertFalse(runner.step());
        assertTrue(runner.step());
        assertEquals(List.of("fast", "slow", "slow"), events);
        assertEquals(java.util.Set.of("drive", "arm"), CommandGroups.parallel("requirements", fast, slow).requirements());
    }

    @Test
    void groupsRejectEmptyChildren() {
        assertThrows(IllegalArgumentException.class, () -> CommandGroups.sequence("empty", List.of()));
        assertThrows(IllegalArgumentException.class, () -> CommandGroups.parallel("empty", List.of()));
    }
}
