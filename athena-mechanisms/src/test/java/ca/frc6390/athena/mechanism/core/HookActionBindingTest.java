package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.HookBinding.HookAction;
import ca.frc6390.athena.mechanism.core.HookBinding.Phase;
import org.junit.jupiter.api.Test;

/**
 * An Action bound to a hook is stored as itself, under the matching phase, through every hook method on
 * both {@link HookBinding} and {@link EventBinding}. Action resolves to the DeviceAction overloads.
 */
class HookActionBindingTest {
    private static final MotorDevice MOTOR = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);

    @Test
    void hookBindingStoresAnActionItselfUnderEachPhase() {
        EventBinding event = Events.when(() -> true).active();
        Action action = MOTOR.percent(0.2);

        assertBound(new HookBinding(event).onStart(action), Phase.ON_START, action);
        assertBound(new HookBinding(event).whileActive(action), Phase.WHILE_ACTIVE, action);
        assertBound(new HookBinding(event).onEnd(action), Phase.ON_END, action);
        assertBound(new HookBinding(event).onInactive(action), Phase.ON_INACTIVE, action);
        assertBound(new HookBinding(event).whileInactive(action), Phase.WHILE_INACTIVE, action);
    }

    @Test
    void eventBindingDefaultsStoreAnActionItselfUnderEachPhase() {
        EventBinding event = Events.when(() -> true).active();
        Action action = MOTOR.percent(0.2);

        assertBound(event.onStart(action), Phase.ON_START, action);
        assertBound(event.whileActive(action), Phase.WHILE_ACTIVE, action);
        assertBound(event.onEnd(action), Phase.ON_END, action);
        assertBound(event.onInactive(action), Phase.ON_INACTIVE, action);
        assertBound(event.whileInactive(action), Phase.WHILE_INACTIVE, action);
    }

    @Test
    void runnableAndDeviceActionOverloadsStillBindTheSameWay() {
        EventBinding event = Events.when(() -> true).active();
        Action action = MOTOR.percent(0.2);
        Runnable runnable = () -> { };

        HookBinding asDevice = new HookBinding(event).onStart((ca.frc6390.athena.hardware.runtime.DeviceAction) action);
        assertBound(asDevice, Phase.ON_START, action);
        HookBinding asRunnable = new HookBinding(event).onStart(runnable);
        assertEquals(Phase.ON_START, asRunnable.actions().get(0).phase());
    }

    private static void assertBound(HookBinding hook, Phase phase, Action action) {
        assertEquals(1, hook.actions().size());
        HookAction bound = hook.actions().get(0);
        assertEquals(phase, bound.phase());
        assertSame(action, bound.action());
    }
}
