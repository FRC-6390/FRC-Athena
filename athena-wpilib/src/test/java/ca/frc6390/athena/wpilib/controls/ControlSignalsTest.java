package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.EventContext;
import java.time.Duration;
import org.junit.jupiter.api.Test;

class ControlSignalsTest {
    @Test
    void recognizesTimedButtonChords() {
        boolean[] firstRaw = {false};
        boolean[] secondRaw = {false};
        ButtonSignal first = new ButtonSignal("first", () -> firstRaw[0]);
        ButtonSignal second = new ButtonSignal("second", () -> secondRaw[0]);
        ControlSignal chord = ControlSignals.chord(first, second)
                .within(Duration.ofMillis(200));

        assertFalse(ControlSignalTest.sample(chord, 0.0));
        firstRaw[0] = true;
        assertFalse(ControlSignalTest.sample(chord, 0.1));
        secondRaw[0] = true;
        assertTrue(ControlSignalTest.sample(chord, 0.25));
        assertFalse(ControlSignalTest.sample(chord, 0.27));

        firstRaw[0] = false;
        secondRaw[0] = false;
        ControlSignalTest.sample(chord, 0.3);
        firstRaw[0] = true;
        ControlSignalTest.sample(chord, 0.4);
        secondRaw[0] = true;
        assertFalse(ControlSignalTest.sample(chord, 0.7));
    }

    @Test
    void recognizesOrderedPulseSequences() {
        boolean[] firstRaw = {false};
        boolean[] secondRaw = {false};
        ButtonSignal first = new ButtonSignal("first", () -> firstRaw[0]);
        ButtonSignal second = new ButtonSignal("second", () -> secondRaw[0]);
        ControlSignal firstPress = first.pressed();
        ControlSignal firstRelease = first.released();
        ControlSignal secondPress = second.pressed();
        ControlSignal sequence = ControlSignals.sequence(firstPress, firstRelease, secondPress)
                .within(Duration.ofSeconds(1));

        assertFalse(ControlSignalTest.sample(sequence, 0.0));
        firstRaw[0] = true;
        assertFalse(ControlSignalTest.sample(sequence, 0.1));
        firstRaw[0] = false;
        assertFalse(ControlSignalTest.sample(sequence, 0.2));
        secondRaw[0] = true;
        assertTrue(ControlSignalTest.sample(sequence, 0.3));
        assertFalse(ControlSignalTest.sample(sequence, 0.32));
    }

    @Test
    void sharedClickRangesObserveTheSameCompletedCount() {
        boolean[] raw = {false};
        ButtonSignal button = new ButtonSignal("test", () -> raw[0]);
        ClickSequence clicks = button.clicks(Duration.ofMillis(100));
        ControlSignal exact = clicks.exactly(3);
        ControlSignal range = clicks.between(2, 4);
        ControlSignal atLeast = clicks.atLeast(3);

        sampleAll(0.0, exact, range, atLeast);
        for (int index = 0; index < 3; index++) {
            raw[0] = true;
            sampleAll(0.1 + index * 0.1, exact, range, atLeast);
            raw[0] = false;
            sampleAll(0.15 + index * 0.1, exact, range, atLeast);
        }

        EventContext completed = ControlSignalTest.context(0.46, true);
        assertTrue(exact.sample(completed));
        assertTrue(range.sample(completed));
        assertTrue(atLeast.sample(completed));
    }

    private static void sampleAll(double now, ControlSignal... signals) {
        EventContext context = ControlSignalTest.context(now, true);
        for (ControlSignal signal : signals) {
            signal.sample(context);
        }
    }
}
