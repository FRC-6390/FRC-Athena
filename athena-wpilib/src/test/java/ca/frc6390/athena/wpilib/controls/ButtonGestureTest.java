package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.EventContext;
import java.time.Duration;
import org.junit.jupiter.api.Test;

class ButtonGestureTest {
    @Test
    void emitsPressReleaseShortPressAndHoldGestures() {
        boolean[] raw = {false};
        ButtonSignal button = new ButtonSignal("test", () -> raw[0]);
        ControlSignal pressed = button.pressed();
        ControlSignal released = button.released();
        ControlSignal held = button.heldFor(Duration.ofMillis(500));
        ControlSignal holdStarted = button.holdStarted(Duration.ofMillis(500));
        ControlSignal shortPress = button.shortPress(Duration.ofMillis(250));

        sampleAll(0.0, pressed, released, held, holdStarted, shortPress);
        raw[0] = true;
        assertTrue(ControlSignalTest.sample(pressed, 0.1));
        assertFalse(ControlSignalTest.sample(released, 0.1));
        assertFalse(ControlSignalTest.sample(held, 0.1));
        assertFalse(ControlSignalTest.sample(holdStarted, 0.1));
        assertFalse(ControlSignalTest.sample(shortPress, 0.1));

        assertTrue(ControlSignalTest.sample(held, 0.6));
        assertTrue(ControlSignalTest.sample(holdStarted, 0.6));
        assertFalse(ControlSignalTest.sample(holdStarted, 0.62));

        raw[0] = false;
        assertTrue(ControlSignalTest.sample(released, 0.7));
        assertFalse(ControlSignalTest.sample(shortPress, 0.7));

        raw[0] = true;
        sampleAll(1.0, pressed, released, held, holdStarted, shortPress);
        raw[0] = false;
        assertTrue(ControlSignalTest.sample(shortPress, 1.2));
    }

    @Test
    void exactClickCountsAreDisambiguatedAfterTheWindow() {
        boolean[] raw = {false};
        ButtonSignal button = new ButtonSignal("test", () -> raw[0]);
        ClickSequence clicks = button.clicks(Duration.ofMillis(300));
        ControlSignal single = clicks.exactly(1);
        ControlSignal twice = clicks.exactly(2);

        sampleBoth(single, twice, 0.0);
        raw[0] = true;
        sampleBoth(single, twice, 0.05);
        raw[0] = false;
        sampleBoth(single, twice, 0.10);
        assertTrue(clicks.pending());
        raw[0] = true;
        sampleBoth(single, twice, 0.20);
        raw[0] = false;
        sampleBoth(single, twice, 0.25);

        assertFalse(ControlSignalTest.sample(single, 0.54));
        assertFalse(ControlSignalTest.sample(twice, 0.54));
        EventContext completed = ControlSignalTest.context(0.551, true);
        assertFalse(single.sample(completed));
        assertTrue(twice.sample(completed));
        assertFalse(clicks.pending());
        assertFalse(ControlSignalTest.sample(twice, 0.57));
    }

    @Test
    void repeatPulsesImmediatelyAndAtConfiguredTimes() {
        boolean[] raw = {false};
        ButtonSignal button = new ButtonSignal("test", () -> raw[0]);
        ControlSignal repeat = button.repeated(
                Duration.ofMillis(400),
                Duration.ofMillis(100));

        assertFalse(ControlSignalTest.sample(repeat, 0.0));
        raw[0] = true;
        assertTrue(ControlSignalTest.sample(repeat, 0.1));
        assertFalse(ControlSignalTest.sample(repeat, 0.49));
        assertTrue(ControlSignalTest.sample(repeat, 0.5));
        assertTrue(ControlSignalTest.sample(repeat, 0.6));
        raw[0] = false;
        assertFalse(ControlSignalTest.sample(repeat, 0.62));
    }

    private static void sampleAll(double now, ControlSignal... signals) {
        for (ControlSignal signal : signals) {
            ControlSignalTest.sample(signal, now);
        }
    }

    private static void sampleBoth(ControlSignal first, ControlSignal second, double now) {
        EventContext context = ControlSignalTest.context(now, true);
        first.sample(context);
        second.sample(context);
    }
}
