package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.EventContext;
import java.time.Duration;
import org.junit.jupiter.api.Test;

class ControlSignalEdgeCaseTest {
    @Test
    void booleanOperatorsCoverEveryTruthTableRow() {
        boolean[] left = {false};
        boolean[] right = {false};
        ButtonSignal signal = new ButtonSignal("left", () -> left[0]);
        ControlSignal and = signal.and(() -> right[0]);
        ControlSignal or = signal.or(() -> right[0]);
        ControlSignal xor = signal.xor(() -> right[0]);
        ControlSignal unless = signal.unless(() -> right[0]);
        ControlSignal not = signal.negate();

        for (int bits = 0; bits < 4; bits++) {
            left[0] = (bits & 2) != 0;
            right[0] = (bits & 1) != 0;
            EventContext context = ControlSignalTest.context(bits, true);
            assertTrue(and.sample(context) == (left[0] && right[0]));
            assertTrue(or.sample(context) == (left[0] || right[0]));
            assertTrue(xor.sample(context) == (left[0] ^ right[0]));
            assertTrue(unless.sample(context) == (left[0] && !right[0]));
            assertTrue(not.sample(context) == !left[0]);
        }
    }

    @Test
    void aRejectedPressIsNotReplayedWhenItsConditionLaterBecomesTrue() {
        boolean[] raw = {false};
        boolean[] allowed = {false};
        ButtonSignal button = new ButtonSignal("test", () -> raw[0]);
        ControlSignal guardedPress = button.pressed().onlyIf(() -> allowed[0]);

        assertFalse(ControlSignalTest.sample(guardedPress, 0.0));
        raw[0] = true;
        assertFalse(ControlSignalTest.sample(guardedPress, 0.02));
        allowed[0] = true;
        assertFalse(ControlSignalTest.sample(guardedPress, 0.04));
        raw[0] = false;
        assertFalse(ControlSignalTest.sample(guardedPress, 0.06));
        raw[0] = true;
        assertTrue(ControlSignalTest.sample(guardedPress, 0.08));
    }

    @Test
    void debounceRequiresAFullStableInitialRisingPeriodAndRestartsAfterChatter() {
        boolean[] raw = {true};
        ControlSignal debounced = new ButtonSignal("test", () -> raw[0])
                .debounce(Duration.ofMillis(100));

        assertFalse(ControlSignalTest.sample(debounced, 0.0));
        assertFalse(ControlSignalTest.sample(debounced, 0.099));
        raw[0] = false;
        assertFalse(ControlSignalTest.sample(debounced, 0.10));
        raw[0] = true;
        assertFalse(ControlSignalTest.sample(debounced, 0.15));
        assertFalse(ControlSignalTest.sample(debounced, 0.249));
        assertTrue(ControlSignalTest.sample(debounced, 0.251));
    }

    @Test
    void zeroDebounceAppliesTransitionsOnTheObservingTick() {
        boolean[] raw = {false};
        ControlSignal debounced = new ButtonSignal("test", () -> raw[0])
                .debounce(Duration.ZERO);

        assertFalse(ControlSignalTest.sample(debounced, 0.0));
        raw[0] = true;
        assertTrue(ControlSignalTest.sample(debounced, 0.02));
        raw[0] = false;
        assertFalse(ControlSignalTest.sample(debounced, 0.04));
    }

    @Test
    void durationTransformsRejectNullAndNegativeValues() {
        ButtonSignal button = new ButtonSignal("test", () -> false);

        assertThrows(NullPointerException.class, () -> button.debounce(null));
        assertThrows(IllegalArgumentException.class, () -> button.debounce(Duration.ofNanos(-1)));
        assertThrows(NullPointerException.class, () -> button.debounce(Duration.ZERO, null));
    }

    @Test
    void toggleSupportsInitialManualConditionalAndResetPolicies() {
        boolean[] raw = {false};
        boolean[] set = {false};
        boolean[] clear = {false};
        boolean[] reset = {false};
        ToggleSignal toggle = new ButtonSignal("test", () -> raw[0])
                .toggle(true)
                .setWhen(() -> set[0])
                .clearWhen(() -> clear[0])
                .resetWhen(() -> reset[0]);

        assertTrue(ControlSignalTest.sample(toggle, 0.0));
        assertFalse(toggle.disable().state());
        assertTrue(toggle.enable().state());
        assertFalse(toggle.flip().state());
        assertTrue(toggle.set(true).state());

        clear[0] = true;
        assertFalse(ControlSignalTest.sample(toggle, 0.02));
        clear[0] = false;
        set[0] = true;
        assertTrue(ControlSignalTest.sample(toggle, 0.04));
        set[0] = false;
        toggle.disable();
        reset[0] = true;
        assertTrue(ControlSignalTest.sample(toggle, 0.06));

        reset[0] = false;
        toggle.disable().resetOnDisable(false);
        assertFalse(ControlSignalTest.sample(toggle, 0.08, false));
        toggle.enable().resetOnDisable(true);
        assertTrue(ControlSignalTest.sample(toggle, 0.10, false));
    }

    @Test
    void clearWinsWhenSetAndClearConditionsAreSimultaneouslyActive() {
        ToggleSignal toggle = new ButtonSignal("test", () -> false)
                .toggle()
                .setWhen(() -> true)
                .clearWhen(() -> true);

        assertFalse(ControlSignalTest.sample(toggle, 0.0));
    }
}
