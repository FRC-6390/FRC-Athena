package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.EventContext;
import java.time.Duration;
import org.junit.jupiter.api.Test;

class ButtonGestureEdgeCaseTest {
    @Test
    void edgeSignalsDoNotInventAReleaseAtStartupAndRearmAfterEveryCycle() {
        boolean[] raw = {false};
        ButtonSignal button = new ButtonSignal("test", () -> raw[0]);
        ControlSignal pressed = button.pressed();
        ControlSignal released = button.released();

        sampleBoth(pressed, released, 0.0, false, false);
        raw[0] = true;
        sampleBoth(pressed, released, 0.02, true, false);
        sampleBoth(pressed, released, 0.04, false, false);
        raw[0] = false;
        sampleBoth(pressed, released, 0.06, false, true);
        sampleBoth(pressed, released, 0.08, false, false);
        raw[0] = true;
        sampleBoth(pressed, released, 0.10, true, false);
    }

    @Test
    void holdBoundaryReleaseAndRepressAreHandledIndependently() {
        boolean[] raw = {false};
        ButtonSignal button = new ButtonSignal("test", () -> raw[0]);
        ControlSignal held = button.heldFor(Duration.ofMillis(500));

        assertFalse(ControlSignalTest.sample(held, 0.0));
        raw[0] = true;
        assertFalse(ControlSignalTest.sample(held, 1.0));
        assertFalse(ControlSignalTest.sample(held, 1.499));
        assertTrue(ControlSignalTest.sample(held, 1.501));
        raw[0] = false;
        assertFalse(ControlSignalTest.sample(held, 1.52));
        raw[0] = true;
        assertFalse(ControlSignalTest.sample(held, 2.0));
        assertTrue(ControlSignalTest.sample(held, 2.501));
    }

    @Test
    void holdStartedEmitsOnlyOncePerPhysicalHold() {
        boolean[] raw = {false};
        ControlSignal started = new ButtonSignal("test", () -> raw[0])
                .holdStarted(Duration.ofMillis(100));

        assertFalse(ControlSignalTest.sample(started, 0.0));
        raw[0] = true;
        assertFalse(ControlSignalTest.sample(started, 0.1));
        assertTrue(ControlSignalTest.sample(started, 0.201));
        assertFalse(ControlSignalTest.sample(started, 0.3));
        raw[0] = false;
        assertFalse(ControlSignalTest.sample(started, 0.4));
        raw[0] = true;
        assertFalse(ControlSignalTest.sample(started, 0.5));
        assertTrue(ControlSignalTest.sample(started, 0.601));
    }

    @Test
    void shortPressAcceptsTheLimitAndRejectsLongerOrUnpairedReleases() {
        boolean[] raw = {false};
        ControlSignal shortPress = new ButtonSignal("test", () -> raw[0])
                .shortPress(Duration.ofMillis(250));

        assertFalse(ControlSignalTest.sample(shortPress, 0.0));
        raw[0] = true;
        assertFalse(ControlSignalTest.sample(shortPress, 1.0));
        raw[0] = false;
        assertTrue(ControlSignalTest.sample(shortPress, 1.25));
        raw[0] = true;
        ControlSignalTest.sample(shortPress, 2.0);
        raw[0] = false;
        assertFalse(ControlSignalTest.sample(shortPress, 2.251));
        assertFalse(ControlSignalTest.sample(shortPress, 2.3));
    }

    @Test
    void repeatRearmsAndDoesNotBurstAfterALargeTimeJump() {
        boolean[] raw = {false};
        ControlSignal repeat = new ButtonSignal("test", () -> raw[0])
                .repeated(Duration.ofMillis(100), Duration.ofMillis(50));

        ControlSignalTest.sample(repeat, 0.0);
        raw[0] = true;
        assertTrue(ControlSignalTest.sample(repeat, 0.1));
        assertTrue(ControlSignalTest.sample(repeat, 1.0));
        assertFalse(ControlSignalTest.sample(repeat, 1.01));
        raw[0] = false;
        assertFalse(ControlSignalTest.sample(repeat, 1.02));
        raw[0] = true;
        assertTrue(ControlSignalTest.sample(repeat, 1.03));
        assertFalse(ControlSignalTest.sample(repeat, 1.12));
        assertTrue(ControlSignalTest.sample(repeat, 1.131));
    }

    @Test
    void clickWindowBoundaryLongPressCancellationAndCountTelemetryAreStable() {
        boolean[] raw = {false};
        ButtonSignal button = new ButtonSignal("test", () -> raw[0]);
        ClickSequence clicks = button.clicks(Duration.ofMillis(100));
        ControlSignal single = clicks.exactly(1);

        assertFalse(ControlSignalTest.sample(single, 0.0));
        raw[0] = true;
        ControlSignalTest.sample(single, 0.1);
        raw[0] = false;
        ControlSignalTest.sample(single, 0.2);
        assertTrue(clicks.pending());
        assertFalse(ControlSignalTest.sample(single, 0.299));
        assertTrue(ControlSignalTest.sample(single, 0.301));
        assertEquals(1, clicks.count().getAsInt());
        assertFalse(clicks.pending());
        assertFalse(ControlSignalTest.sample(single, 0.32));
        assertEquals(0, clicks.count().getAsInt());

        raw[0] = true;
        ControlSignalTest.sample(single, 1.0);
        raw[0] = false;
        ControlSignalTest.sample(single, 1.1);
        raw[0] = true;
        ControlSignalTest.sample(single, 1.15);
        raw[0] = false;
        assertFalse(ControlSignalTest.sample(single, 1.7));
        assertFalse(clicks.pending());
        assertFalse(ControlSignalTest.sample(single, 1.9));
    }

    @Test
    void recognizerHandlesAHundredClicksWithoutOverflowOrLostEdges() {
        boolean[] raw = {false};
        ButtonSignal button = new ButtonSignal("test", () -> raw[0]);
        ClickSequence clicks = button.clicks(Duration.ofMillis(10));
        ControlSignal hundred = clicks.exactly(100);

        ControlSignalTest.sample(hundred, 0.0);
        for (int index = 0; index < 100; index++) {
            raw[0] = true;
            ControlSignalTest.sample(hundred, 0.01 + index * 0.002);
            raw[0] = false;
            ControlSignalTest.sample(hundred, 0.011 + index * 0.002);
        }
        assertTrue(ControlSignalTest.sample(hundred, 0.25));
        assertEquals(100, clicks.count().getAsInt());
    }

    @Test
    void gestureArgumentsRejectInvalidValues() {
        ButtonSignal button = new ButtonSignal("test", () -> false);
        ClickSequence clicks = button.clicks(Duration.ofMillis(100));

        assertThrows(IllegalArgumentException.class, () -> button.heldFor(Duration.ofNanos(-1)));
        assertThrows(NullPointerException.class, () -> button.shortPress(null));
        assertThrows(IllegalArgumentException.class,
                () -> button.repeated(Duration.ZERO, Duration.ZERO));
        assertThrows(IllegalArgumentException.class,
                () -> button.repeated(Duration.ofMillis(-1), Duration.ofMillis(1)));
        assertThrows(IllegalArgumentException.class, () -> button.clicks(0));
        assertThrows(IllegalArgumentException.class, () -> clicks.exactly(0));
        assertThrows(IllegalArgumentException.class, () -> clicks.atLeast(-1));
        assertThrows(IllegalArgumentException.class, () -> clicks.between(0, 1));
        assertThrows(IllegalArgumentException.class, () -> clicks.between(3, 2));
        assertThrows(IllegalArgumentException.class, () -> button.clicks(Duration.ofMillis(-1)));
    }

    private static void sampleBoth(
            ControlSignal pressed,
            ControlSignal released,
            double now,
            boolean expectedPressed,
            boolean expectedReleased) {
        EventContext context = ControlSignalTest.context(now, true);
        assertTrue(pressed.sample(context) == expectedPressed);
        assertTrue(released.sample(context) == expectedReleased);
    }
}
