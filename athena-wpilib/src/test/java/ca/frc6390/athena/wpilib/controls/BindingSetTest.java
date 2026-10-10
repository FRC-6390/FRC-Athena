package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.EventContext;
import ca.frc6390.athena.mechanism.core.HookBinding;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Test;

/** Tests for {@code in(condition, bindings)} binding sets and their gating of derived signals. */
class BindingSetTest {
    private static final double TICK = 0.02;

    @Test
    void gatedButtonIsActiveOnlyWhileConditionHoldsAndRootBindingsAreUnaffected() {
        FakeInput input = new FakeInput();
        Gamepad pad = new Gamepad(input);
        boolean[] armed = {false};
        ButtonSignal[] gated = new ButtonSignal[1];
        pad.in(() -> armed[0], view -> gated[0] = view.a());
        ButtonSignal root = pad.a();
        input.buttons[1] = true;

        assertFalse(active(gated[0], 0.0));
        assertTrue(active(root, 0.0));

        armed[0] = true;
        assertTrue(active(gated[0], TICK));
        assertTrue(active(root, TICK));
    }

    @Test
    void conditionTurningFalseWhileHeldReleasesTheBinding() {
        FakeInput input = new FakeInput();
        Gamepad pad = new Gamepad(input);
        boolean[] armed = {true};
        ButtonSignal[] gated = new ButtonSignal[1];
        pad.in(() -> armed[0], view -> {
            gated[0] = view.a();
            gated[0].whileHeld(() -> {}).onRelease(() -> {});
        });
        input.buttons[1] = true;

        assertTrue(active(gated[0], 0.0));
        armed[0] = false;
        assertFalse(active(gated[0], TICK));
        assertTrue(hasPhase(gated[0], HookBinding.Phase.WHILE_ACTIVE));
        assertTrue(hasPhase(gated[0], HookBinding.Phase.ON_END));
    }

    @Test
    void nestedSetsCombineTheirConditions() {
        FakeInput input = new FakeInput();
        Gamepad pad = new Gamepad(input);
        boolean[] outer = {false};
        boolean[] inner = {false};
        ButtonSignal[] signal = new ButtonSignal[1];
        pad.in(() -> outer[0], outerView -> outerView.in(() -> inner[0], innerView -> signal[0] = innerView.b()));
        input.buttons[2] = true;

        assertFalse(active(signal[0], 0.0));
        inner[0] = true;
        assertFalse(active(signal[0], TICK));
        outer[0] = true;
        assertTrue(active(signal[0], 2 * TICK));
        inner[0] = false;
        assertFalse(active(signal[0], 3 * TICK));
    }

    @Test
    void gatedBindingsAreRegisteredOnceWithTheRootGamepad() {
        Gamepad pad = new Gamepad(new FakeInput());
        pad.a().onPress(() -> {});
        pad.in(() -> true, view -> view.b().whileHeld(() -> {}));
        pad.in(() -> true, view -> view.in(() -> true, inner -> inner.x().onPress(() -> {})));

        assertEquals(3, pad.hooks().size());
    }

    @Test
    void derivedPulsesInsideASetAreGated() {
        FakeInput input = new FakeInput();
        Gamepad pad = new Gamepad(input);
        boolean[] armed = {false};
        ControlSignal[] pressed = new ControlSignal[1];
        pad.in(() -> armed[0], view -> pressed[0] = view.a().pressed());
        input.buttons[1] = true;

        assertFalse(active(pressed[0], 0.0));
        armed[0] = true;
        assertTrue(active(pressed[0], TICK));
        assertFalse(active(pressed[0], 2 * TICK));
        armed[0] = false;
        assertFalse(active(pressed[0], 3 * TICK));
        armed[0] = true;
        // The button is still held, but re-entering the set is a new press for the gated binding.
        assertTrue(active(pressed[0], 4 * TICK));
    }

    @Test
    void negatedSignalInsideASetIsFalseWhileTheSetIsInactive() {
        FakeInput input = new FakeInput();
        Gamepad pad = new Gamepad(input);
        boolean[] armed = {false};
        ControlSignal[] negated = new ControlSignal[1];
        pad.in(() -> armed[0], view -> negated[0] = view.a().negate());

        assertFalse(active(negated[0], 0.0));
        armed[0] = true;
        assertTrue(active(negated[0], TICK));
    }

    @Test
    void toggleLatchesWhileGatedButOnlyFlipsOnRisingEdgesInsideTheSet() {
        FakeInput input = new FakeInput();
        Gamepad pad = new Gamepad(input);
        boolean[] armed = {false};
        ToggleSignal[] toggle = new ToggleSignal[1];
        pad.in(() -> armed[0], view -> toggle[0] = view.a().toggle());

        input.buttons[1] = true;
        assertFalse(active(toggle[0], 0.0));
        assertFalse(toggle[0].state());
        input.buttons[1] = false;
        armed[0] = true;
        assertFalse(active(toggle[0], TICK));
        input.buttons[1] = true;
        assertTrue(active(toggle[0], 2 * TICK));
        assertTrue(toggle[0].state());

        // Disarming hides the output and keeps the latched state while inactive.
        armed[0] = false;
        assertFalse(active(toggle[0], 3 * TICK));
        assertTrue(toggle[0].state());
        // The button is still held when the set re-enters, which is a new press and flips the toggle.
        armed[0] = true;
        assertFalse(active(toggle[0], 4 * TICK));
        assertFalse(toggle[0].state());
    }

    @Test
    void clickSequenceEmissionIsSuppressedWhileTheSetIsInactive() {
        FakeInput input = new FakeInput();
        Gamepad pad = new Gamepad(input);
        boolean[] armed = {true};
        ControlSignal[] doubleClick = new ControlSignal[1];
        pad.in(() -> armed[0], view -> doubleClick[0] = view.a().doubleClick());

        clickOnce(input, doubleClick[0], 0.0);
        clickOnce(input, doubleClick[0], 0.2);
        armed[0] = false;
        assertFalse(active(doubleClick[0], 0.65));
    }

    @Test
    void doubleClickEmitsWhenTheSetStaysActive() {
        FakeInput input = new FakeInput();
        Gamepad pad = new Gamepad(input);
        ControlSignal[] doubleClick = new ControlSignal[1];
        pad.in(() -> true, view -> doubleClick[0] = view.a().doubleClick());

        clickOnce(input, doubleClick[0], 0.0);
        clickOnce(input, doubleClick[0], 0.2);
        assertTrue(active(doubleClick[0], 0.65));
    }

    @Test
    void repeatedPulsesInsideASetStopWhileTheSetIsInactive() {
        FakeInput input = new FakeInput();
        Gamepad pad = new Gamepad(input);
        boolean[] armed = {true};
        ControlSignal[] repeat = new ControlSignal[1];
        pad.in(() -> armed[0], view -> repeat[0] = view.a().repeated(
                java.time.Duration.ofMillis(400), java.time.Duration.ofMillis(100)));
        input.buttons[1] = true;

        assertTrue(active(repeat[0], 0.0));
        assertFalse(active(repeat[0], 0.1));
        assertTrue(active(repeat[0], 0.4));
        armed[0] = false;
        assertFalse(active(repeat[0], 0.5));
        armed[0] = true;
        // Re-entering while held is a new press, so the repeat sequence restarts immediately.
        assertTrue(active(repeat[0], 0.6));
    }

    @Test
    void onlyIfInsideASetAddsAConditionToTheGate() {
        FakeInput input = new FakeInput();
        Gamepad pad = new Gamepad(input);
        boolean[] outer = {true};
        boolean[] inner = {false};
        ControlSignal[] guarded = new ControlSignal[1];
        pad.in(() -> outer[0], view -> guarded[0] = view.a().onlyIf(() -> inner[0]));
        input.buttons[1] = true;

        assertFalse(active(guarded[0], 0.0));
        inner[0] = true;
        assertTrue(active(guarded[0], TICK));
        outer[0] = false;
        assertFalse(active(guarded[0], 2 * TICK));
    }

    @Test
    void axisThresholdsInsideASetAreGated() {
        FakeInput input = new FakeInput();
        Gamepad pad = new Gamepad(input);
        boolean[] armed = {false};
        ControlSignal[] trigger = new ControlSignal[1];
        pad.in(() -> armed[0], view -> trigger[0] = view.rightTrigger().above(0.5));
        input.axes[XboxController.Axis.kRightTrigger.value] = 0.9;

        assertFalse(active(trigger[0], 0.0));
        armed[0] = true;
        assertTrue(active(trigger[0], TICK));
    }

    @Test
    void playStationViewExposesPlayStationInputsThroughInPlayStation() {
        FakeInput input = new FakeInput();
        PlayStationGamepad pad = new PlayStationGamepad(input, false);
        boolean[] armed = {false};
        ButtonSignal[] touchpad = new ButtonSignal[1];
        pad.inPlayStation(() -> armed[0], view -> touchpad[0] = view.touchpad());
        input.buttons[PS4Controller.Button.kTouchpad.value] = true;

        assertFalse(active(touchpad[0], 0.0));
        armed[0] = true;
        assertTrue(active(touchpad[0], TICK));
        assertTrue(active(pad.touchpad(), TICK));
    }

    @Test
    void genericControllerViewGatesRawButtonsOnly() {
        GenericFakeInput input = new GenericFakeInput();
        GenericController custom = new GenericController(input);
        boolean[] armed = {false};
        ButtonSignal[] gated = new ButtonSignal[1];
        custom.in(() -> armed[0], view -> gated[0] = view.button(3));
        input.buttons[3] = true;

        assertFalse(active(gated[0], 0.0));
        assertTrue(active(custom.button(3), 0.0));
        armed[0] = true;
        assertTrue(active(gated[0], TICK));
    }

    @Test
    void simultaneousGatedPressesOnOneInputAreReported() {
        List<String> messages = new ArrayList<>();
        FakeInput input = new FakeInput();
        Gamepad pad = new Gamepad(input, messages::add);
        ButtonSignal first = gatedPress(pad, () -> true, () -> {});
        ButtonSignal second = gatedPress(pad, () -> true, () -> {});
        input.buttons[1] = true;

        sampleSameCycle(0.0, first, second);

        assertEquals(1, messages.size());
        assertTrue(messages.get(0).contains("'a'"));
        assertTrue(messages.get(0).contains("PRESS"));
    }

    @Test
    void exclusiveGatedPressesAreNotReported() {
        List<String> messages = new ArrayList<>();
        FakeInput input = new FakeInput();
        Gamepad pad = new Gamepad(input, messages::add);
        boolean[] first = {true};
        boolean[] second = {false};
        ButtonSignal firstSignal = gatedPress(pad, () -> first[0], () -> {});
        ButtonSignal secondSignal = gatedPress(pad, () -> second[0], () -> {});
        input.buttons[1] = true;

        sampleSameCycle(0.0, firstSignal, secondSignal);

        assertTrue(messages.isEmpty());
    }

    @Test
    void conflictsOnDifferentEdgesAreNotReported() {
        List<String> messages = new ArrayList<>();
        FakeInput input = new FakeInput();
        Gamepad pad = new Gamepad(input, messages::add);
        ButtonSignal press = gatedPress(pad, () -> true, () -> {});
        ButtonSignal held = gatedHeld(pad);
        input.buttons[1] = true;

        sampleSameCycle(0.0, press, held);

        assertTrue(messages.isEmpty());
    }

    @Test
    void conflictReportsAreRateLimited() {
        List<String> messages = new ArrayList<>();
        FakeInput input = new FakeInput();
        Gamepad pad = new Gamepad(input, messages::add);
        ButtonSignal first = gatedHeld(pad);
        ButtonSignal second = gatedHeld(pad);
        input.buttons[1] = true;

        for (int tick = 0; tick <= 255; tick++) {
            sampleSameCycle(tick * TICK, first, second);
            if (tick == 245) {
                assertEquals(1, messages.size());
            }
        }
        assertEquals(2, messages.size());
    }

    private static ButtonSignal gatedPress(Gamepad pad, java.util.function.BooleanSupplier condition, Runnable action) {
        ButtonSignal[] signal = new ButtonSignal[1];
        pad.in(condition, view -> {
            signal[0] = view.a();
            signal[0].onPress(action);
        });
        return signal[0];
    }

    private static ButtonSignal gatedHeld(Gamepad pad) {
        ButtonSignal[] signal = new ButtonSignal[1];
        pad.in(() -> true, view -> {
            signal[0] = view.a();
            signal[0].whileHeld(() -> {});
        });
        return signal[0];
    }

    /** Samples every signal against one shared runtime context, as the runtime does within a cycle. */
    private static void sampleSameCycle(double now, ControlSignal... signals) {
        EventContext context = ControlSignalTest.context(now, true);
        for (ControlSignal signal : signals) {
            signal.binding().event().sourceActive(context);
        }
    }

    /** Presses and releases button A, sampling the click signal at each edge. */
    private static void clickOnce(FakeInput input, ControlSignal click, double pressTime) {
        input.buttons[1] = true;
        active(click, pressTime);
        input.buttons[1] = false;
        active(click, pressTime + 0.1);
    }

    private static boolean active(ControlSignal signal, double now) {
        return ControlSignalTest.sample(signal, now);
    }

    private static boolean hasPhase(ControlSignal signal, HookBinding.Phase phase) {
        return signal.binding().actions().stream().anyMatch(action -> action.phase() == phase);
    }

    private static final class FakeInput implements Gamepad.Input {
        private final boolean[] buttons = new boolean[16];
        private final double[] axes = new double[6];
        private boolean connected = true;

        @Override
        public double axis(int axis) {
            return axes[axis];
        }

        @Override
        public boolean button(int button) {
            return buttons[button];
        }

        @Override
        public int pov() {
            return -1;
        }

        @Override
        public boolean connected() {
            return connected;
        }
    }

    private static final class GenericFakeInput implements GenericController.Input {
        private final boolean[] buttons = new boolean[16];

        @Override
        public double axis(int id) {
            return 0.0;
        }

        @Override
        public boolean button(int id) {
            return buttons[id];
        }

        @Override
        public int pov() {
            return -1;
        }

        @Override
        public boolean connected() {
            return true;
        }
    }
}
