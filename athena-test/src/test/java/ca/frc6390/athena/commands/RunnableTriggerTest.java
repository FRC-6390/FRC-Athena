package ca.frc6390.athena.commands;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;

final class RunnableTriggerTest {

    @AfterEach
    void cleanupScheduler() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Test
    void onTrueFiresWhenSignalStartsTrue() {
        AtomicBoolean signal = new AtomicBoolean(true);
        AtomicInteger fired = new AtomicInteger(0);
        new RunnableTrigger(signal::get)
                .onTrue(Commands.runOnce(fired::incrementAndGet).ignoringDisable(true));

        runSchedulerCycles(3);

        assertEquals(1, fired.get());
        signal.set(false);
        runSchedulerCycles(2);
    }

    @Test
    void onTrueStillFiresOnLaterRisingEdgesAfterInitialFire() {
        AtomicBoolean signal = new AtomicBoolean(true);
        AtomicInteger fired = new AtomicInteger(0);
        new RunnableTrigger(signal::get)
                .onTrue(Commands.runOnce(fired::incrementAndGet).ignoringDisable(true));

        runSchedulerCycles(3);
        assertEquals(1, fired.get());

        signal.set(false);
        runSchedulerCycles(2);
        signal.set(true);
        runSchedulerCycles(2);
        assertEquals(2, fired.get());

        signal.set(false);
        runSchedulerCycles(2);
    }

    @Test
    void whileTrueSchedulesAndCancelsWhenSignalStartsTrue() {
        AtomicBoolean signal = new AtomicBoolean(true);
        AtomicInteger cycles = new AtomicInteger(0);
        Command held = Commands.run(cycles::incrementAndGet).ignoringDisable(true);
        CommandScheduler scheduler = CommandScheduler.getInstance();

        new RunnableTrigger(signal::get).whileTrue(held);
        runSchedulerCycles(3);

        assertTrue(scheduler.isScheduled(held));
        int initialCycles = cycles.get();

        runSchedulerCycles(2);
        assertTrue(cycles.get() > initialCycles);

        signal.set(false);
        runSchedulerCycles(2);
        assertFalse(scheduler.isScheduled(held));
    }

    @Test
    void toggleOnTrueTogglesWhenSignalStartsTrue() {
        AtomicBoolean signal = new AtomicBoolean(true);
        AtomicInteger cycles = new AtomicInteger(0);
        Command toggled = Commands.run(cycles::incrementAndGet).ignoringDisable(true);
        CommandScheduler scheduler = CommandScheduler.getInstance();

        new RunnableTrigger(signal::get).toggleOnTrue(toggled);
        runSchedulerCycles(3);
        assertTrue(scheduler.isScheduled(toggled));

        signal.set(false);
        runSchedulerCycles(2);
        signal.set(true);
        runSchedulerCycles(2);
        assertFalse(scheduler.isScheduled(toggled));
    }

    private static void runSchedulerCycles(int cycles) {
        CommandScheduler scheduler = CommandScheduler.getInstance();
        for (int i = 0; i < cycles; i++) {
            scheduler.run();
        }
    }
}
