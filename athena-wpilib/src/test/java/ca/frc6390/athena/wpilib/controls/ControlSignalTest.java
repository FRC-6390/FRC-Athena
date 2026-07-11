package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.LifecycleMode;
import ca.frc6390.athena.mechanism.core.LifecyclePhase;
import java.time.Duration;
import org.junit.jupiter.api.Test;

class ControlSignalTest {
    @Test
    void composesConditionsAndSamplesDependenciesOncePerTick() {
        boolean[] raw = {true};
        int[] samples = {0};
        ButtonSignal button = new ButtonSignal("test", () -> {
            samples[0]++;
            return raw[0];
        });
        ControlSignal composed = button.and(button).unless(() -> false);

        EventContext tick = context(1.0, true);
        assertTrue(composed.binding().event().sourceActive(tick));
        assertTrue(composed.binding().event().sourceActive(tick));
        assertEquals(1, samples[0]);

        raw[0] = false;
        assertFalse(sample(composed, 1.02));
        assertEquals(2, samples[0]);
    }

    @Test
    void debouncesRisingAndFallingTransitionsIndependently() {
        boolean[] raw = {false};
        ButtonSignal button = new ButtonSignal("test", () -> raw[0]);
        ControlSignal debounced = button.debounce(
                Duration.ofMillis(100),
                Duration.ofMillis(200));

        assertFalse(sample(debounced, 0.0));
        raw[0] = true;
        assertFalse(sample(debounced, 0.05));
        assertFalse(sample(debounced, 0.14));
        assertTrue(sample(debounced, 0.151));

        raw[0] = false;
        assertTrue(sample(debounced, 0.20));
        assertTrue(sample(debounced, 0.39));
        assertFalse(sample(debounced, 0.401));
    }

    @Test
    void toggleFlipsOnlyOnRisingEdgesAndResetsWhileDisabled() {
        boolean[] raw = {false};
        ButtonSignal button = new ButtonSignal("test", () -> raw[0]);
        ToggleSignal toggle = button.toggle();

        assertFalse(sample(toggle, 0.0));
        raw[0] = true;
        assertTrue(sample(toggle, 0.02));
        assertTrue(sample(toggle, 0.04));
        raw[0] = false;
        assertTrue(sample(toggle, 0.06));
        raw[0] = true;
        assertFalse(sample(toggle, 0.08));

        raw[0] = false;
        assertFalse(sample(toggle, 0.10, false));
        raw[0] = true;
        assertTrue(sample(toggle, 0.12));
    }

    static boolean sample(ControlSignal signal, double now) {
        return sample(signal, now, true);
    }

    static boolean sample(ControlSignal signal, double now, boolean enabled) {
        return signal.binding().event().sourceActive(context(now, enabled));
    }

    static EventContext context(double now, boolean enabled) {
        return new EventContext(
                now,
                0.02,
                enabled ? LifecycleMode.TELEOP : LifecycleMode.DISABLED,
                LifecyclePhase.PERIODIC,
                enabled,
                false);
    }
}
