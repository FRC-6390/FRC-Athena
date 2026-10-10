package ca.frc6390.athena.mechanism.staging;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import edu.wpi.first.units.Units;
import java.util.ArrayList;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import org.junit.jupiter.api.Test;

class StagingTest {
    enum Health { NORMAL, STRAINED, HOT, SAFE }

    private static final ControlBinding<?> ANGLE = Controls.velocity(MotorDevice.of(MotorKinds.KRAKEN_X60, 61));

    /** Records stage Action requests and cancels in order, and reports gate state. */
    private static final class Host implements StageHost {
        final List<String> log = new ArrayList<>();
        final Map<Action, String> names = new IdentityHashMap<>();
        final Set<Action> running = new HashSet<>();

        String name(Action action) { return names.computeIfAbsent(action, a -> "a" + names.size()); }

        @Override public void request(Action action) { log.add("request " + name(action)); }
        @Override public void cancel(Action action) { log.add("cancel " + name(action)); }
        @Override public boolean isRunning(Action action) { return running.contains(action); }
    }

    private static final class Flags {
        boolean strained;
        boolean hot;
        boolean shutdown;
        boolean override;
    }

    private static Staging<Health> staging(Flags flags, Action hotAction) {
        return Staging.of(Health.NORMAL)
                .stage(Health.STRAINED, () -> flags.strained, ANGLE.limitEffort(0.6))
                .stage(Health.HOT, () -> flags.hot, hotAction)
                .stage(Health.SAFE, () -> flags.shutdown, ANGLE.limitSpeed(0.0))
                .exitAfter(Units.Seconds.of(0.5))
                .override(() -> flags.override);
    }

    @Test
    void worstTrueStageIsEnteredImmediatelyAndCapsFollowStage() {
        Flags flags = new Flags();
        Staging<Health> staging = staging(flags, Actions.waitSeconds(1));
        Host host = new Host();
        flags.strained = true;
        flags.hot = true;
        staging.evaluate(0.0, host);
        assertEquals(Health.HOT, staging.currentStage());
        assertEquals(Health.HOT, staging.current().get());
    }

    @Test
    void exitRequiresConditionFalseForExitAfter() {
        Flags flags = new Flags();
        Staging<Health> staging = staging(flags, Actions.waitSeconds(1));
        Host host = new Host();
        flags.hot = true;
        staging.evaluate(0.0, host);
        flags.hot = false;
        staging.evaluate(1.0, host);
        staging.evaluate(1.3, host);
        assertEquals(Health.HOT, staging.currentStage(), "still inside exit delay");
        staging.evaluate(1.6, host);
        assertEquals(Health.NORMAL, staging.currentStage());
    }

    @Test
    void conditionFlickerRestartsExitDelay() {
        Flags flags = new Flags();
        Staging<Health> staging = staging(flags, Actions.waitSeconds(1));
        Host host = new Host();
        flags.hot = true;
        staging.evaluate(0.0, host);
        flags.hot = false;
        staging.evaluate(1.0, host);
        flags.hot = true;
        staging.evaluate(1.2, host);
        flags.hot = false;
        staging.evaluate(1.3, host);
        staging.evaluate(1.6, host);
        assertEquals(Health.HOT, staging.currentStage(), "delay restarted at 1.3");
        staging.evaluate(1.9, host);
        assertEquals(Health.NORMAL, staging.currentStage());
    }

    @Test
    void worstRemainingTrueStageIsEnteredAfterExitDelay() {
        Flags flags = new Flags();
        Staging<Health> staging = staging(flags, Actions.waitSeconds(1));
        Host host = new Host();
        flags.strained = true;
        flags.hot = true;
        staging.evaluate(0.0, host);
        flags.hot = false;
        staging.evaluate(0.1, host);
        assertEquals(Health.HOT, staging.currentStage());
        staging.evaluate(0.7, host);
        assertEquals(Health.STRAINED, staging.currentStage());
    }

    @Test
    void overrideForcesInitialStageUntilReleased() {
        Flags flags = new Flags();
        Staging<Health> staging = staging(flags, Actions.waitSeconds(1));
        Host host = new Host();
        flags.hot = true;
        flags.override = true;
        staging.evaluate(0.0, host);
        assertEquals(Health.NORMAL, staging.currentStage());
        flags.override = false;
        staging.evaluate(0.1, host);
        assertEquals(Health.HOT, staging.currentStage());
    }

    @Test
    void stageActionsAreRequestedOnEntryAndCancelledOnExit() {
        Flags flags = new Flags();
        Action hotAction = Actions.waitSeconds(1);
        Staging<Health> staging = staging(flags, hotAction);
        Host host = new Host();
        flags.hot = true;
        staging.evaluate(0.0, host);
        flags.hot = false;
        staging.evaluate(1.0, host);
        staging.evaluate(1.6, host);
        assertEquals(List.of("request a0", "cancel a0"), host.log);
        assertEquals(2, staging.drainChanges().size(), "NORMAL to HOT and back");
    }

    @Test
    void modifiersApplyOnlyWhileTheirStageIsCurrent() {
        Flags flags = new Flags();
        Staging<Health> staging = staging(flags, Actions.waitSeconds(1));
        Host host = new Host();
        assertTrue(staging.activeModifiers().isEmpty());
        flags.strained = true;
        staging.evaluate(0.0, host);
        assertEquals(1, staging.activeModifiers().size());
        assertEquals(StageModifier.Kind.EFFORT, staging.activeModifiers().get(0).kind());
        assertEquals(0.6, staging.activeModifiers().get(0).fraction(), 1e-12);
    }

    @Test
    void stageCapsCombineByMinimum() {
        StageCaps caps = StageCaps.NONE.with(StageModifier.Kind.EFFORT, 0.6)
                .with(StageModifier.Kind.EFFORT, 0.8)
                .with(StageModifier.Kind.SPEED, 0.5);
        assertEquals(0.6, caps.effort(), 1e-12);
        assertEquals(0.5, caps.speed(), 1e-12);
        assertEquals(1.0, caps.acceleration(), 1e-12);
    }

    @Test
    void gatedStagingIsActiveOnlyWhileGateActionRuns() {
        Flags flags = new Flags();
        Action gate = Actions.waitSeconds(2);
        Staging<Health> staging = staging(flags, Actions.waitSeconds(1));
        Host host = new Host();
        staging.gate(gate);
        flags.hot = true;
        staging.evaluate(0.0, host);
        assertFalse(staging.active());
        assertEquals(Health.NORMAL, staging.currentStage(), "idle gate keeps stages out");
        host.running.add(gate);
        staging.evaluate(0.1, host);
        assertTrue(staging.active());
        assertEquals(Health.HOT, staging.currentStage());
        host.running.remove(gate);
        staging.evaluate(0.2, host);
        assertEquals(Health.NORMAL, staging.currentStage(), "gate ends: stage resets immediately");
    }

    @Test
    void invalidDeclarationsAreRejected() {
        Flags flags = new Flags();
        Staging<Health> staging = staging(flags, Actions.waitSeconds(1));
        assertThrows(IllegalArgumentException.class,
                () -> staging.stage(Health.HOT, () -> true, Actions.waitSeconds(1)));
        assertThrows(IllegalArgumentException.class,
                () -> staging.stage(Health.NORMAL, () -> true, ANGLE.limitEffort(0.5)));
        assertThrows(IllegalArgumentException.class, () -> ANGLE.limitEffort(1.5));
    }
}
