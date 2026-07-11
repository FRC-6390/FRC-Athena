package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.time.Duration;
import org.junit.jupiter.api.Test;

class ControlSignalsEdgeCaseTest {
    @Test
    void emptyAndGroupedConditionsUseMathematicalIdentityValues() {
        assertTrue(ControlSignalTest.sample(ControlSignals.allOf(), 0.0));
        assertFalse(ControlSignalTest.sample(ControlSignals.anyOf(), 0.0));
        assertTrue(ControlSignalTest.sample(ControlSignals.noneOf(), 0.0));

        boolean[] first = {true};
        boolean[] second = {false};
        assertFalse(ControlSignalTest.sample(
                ControlSignals.allOf(() -> first[0], () -> second[0]), 0.02));
        assertTrue(ControlSignalTest.sample(
                ControlSignals.anyOf(() -> first[0], () -> second[0]), 0.04));
        assertFalse(ControlSignalTest.sample(
                ControlSignals.noneOf(() -> first[0], () -> second[0]), 0.06));
    }

    @Test
    void groupFactoriesRejectNullArraysAndElements() {
        assertThrows(NullPointerException.class,
                () -> ControlSignals.allOf((java.util.function.BooleanSupplier[]) null));
        assertThrows(NullPointerException.class,
                () -> ControlSignals.anyOf(() -> true, null));
        assertThrows(NullPointerException.class,
                () -> ControlSignals.noneOf((java.util.function.BooleanSupplier) null));
    }

    @Test
    void chordAcceptsTheWindowBoundaryDoesNotRepeatAndRearms() {
        boolean[] firstRaw = {false};
        boolean[] secondRaw = {false};
        ButtonSignal first = new ButtonSignal("first", () -> firstRaw[0]);
        ButtonSignal second = new ButtonSignal("second", () -> secondRaw[0]);
        ControlSignal chord = ControlSignals.chord(first, second)
                .within(Duration.ofMillis(200));

        assertFalse(ControlSignalTest.sample(chord, 0.0));
        firstRaw[0] = true;
        assertFalse(ControlSignalTest.sample(chord, 1.0));
        secondRaw[0] = true;
        assertTrue(ControlSignalTest.sample(chord, 1.2));
        assertFalse(ControlSignalTest.sample(chord, 1.21));

        firstRaw[0] = false;
        secondRaw[0] = false;
        assertFalse(ControlSignalTest.sample(chord, 1.22));
        firstRaw[0] = true;
        assertFalse(ControlSignalTest.sample(chord, 1.23));
        secondRaw[0] = true;
        assertTrue(ControlSignalTest.sample(chord, 1.24));
    }

    @Test
    void chordRejectsInvalidConstructionAndDuration() {
        ButtonSignal button = new ButtonSignal("test", () -> false);

        assertThrows(NullPointerException.class,
                () -> ControlSignals.chord((ButtonSignal[]) null));
        assertThrows(IllegalArgumentException.class, () -> ControlSignals.chord(button));
        assertThrows(NullPointerException.class, () -> ControlSignals.chord(button, null));
        assertThrows(IllegalArgumentException.class,
                () -> ControlSignals.chord(button, button).within(Duration.ofNanos(-1)));
    }

    @Test
    void sequenceIgnoresFutureStepsUntilExpectedAndResetsAfterTimeout() {
        boolean[] firstRaw = {false};
        boolean[] secondRaw = {false};
        ButtonSignal first = new ButtonSignal("first", () -> firstRaw[0]);
        ButtonSignal second = new ButtonSignal("second", () -> secondRaw[0]);
        ControlSignal firstPress = first.pressed();
        ControlSignal secondPress = second.pressed();
        ControlSignal sequence = ControlSignals.sequence(firstPress, secondPress)
                .within(Duration.ofMillis(200));

        assertFalse(ControlSignalTest.sample(sequence, 0.0));
        secondRaw[0] = true;
        assertFalse(ControlSignalTest.sample(sequence, 0.02));
        secondRaw[0] = false;
        ControlSignalTest.sample(sequence, 0.04);

        firstRaw[0] = true;
        assertFalse(ControlSignalTest.sample(sequence, 0.10));
        firstRaw[0] = false;
        ControlSignalTest.sample(sequence, 0.12);
        secondRaw[0] = true;
        assertFalse(ControlSignalTest.sample(sequence, 0.31));
        secondRaw[0] = false;
        ControlSignalTest.sample(sequence, 0.32);

        firstRaw[0] = true;
        assertFalse(ControlSignalTest.sample(sequence, 0.40));
        firstRaw[0] = false;
        ControlSignalTest.sample(sequence, 0.42);
        secondRaw[0] = true;
        assertTrue(ControlSignalTest.sample(sequence, 0.59));
        assertFalse(ControlSignalTest.sample(sequence, 0.60));
    }

    @Test
    void sequenceRejectsInvalidConstructionAndDuration() {
        ControlSignal pulse = new ButtonSignal("test", () -> false).pressed();

        assertThrows(NullPointerException.class,
                () -> ControlSignals.sequence((ControlSignal[]) null));
        assertThrows(IllegalArgumentException.class, () -> ControlSignals.sequence(pulse));
        assertThrows(NullPointerException.class, () -> ControlSignals.sequence(pulse, null));
        assertThrows(IllegalArgumentException.class,
                () -> ControlSignals.sequence(pulse, pulse).within(Duration.ofNanos(-1)));
    }
}
