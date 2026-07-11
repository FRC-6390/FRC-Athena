package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Map;
import org.junit.jupiter.api.Test;

class GamepadRegistrationTest {
    @Test
    void physicalAndDerivedBindingsAreDiscoveredExactlyOnce() {
        Gamepad gamepad = new Gamepad(new FakeInput());
        gamepad.a().onPress(() -> {});
        gamepad.b().toggle().onTrue(() -> {});
        gamepad.x().doubleClick().onTrue(() -> {});
        gamepad.rightTrigger().above(0.6, 0.4).onTrue(() -> {});

        Map<String, ?> hooks = gamepad.hooks();

        assertEquals(4, hooks.size());
        assertTrue(hooks.keySet().stream().anyMatch(name -> name.startsWith("a.")));
        assertTrue(hooks.keySet().stream().anyMatch(name -> name.contains("b.toggle")));
        assertTrue(hooks.keySet().stream().anyMatch(name -> name.contains("x.clicks[2]")));
        assertTrue(hooks.keySet().stream().anyMatch(name -> name.contains("rightTrigger.above")));
    }

    @Test
    void unboundIntermediateSignalsDoNotCreateEmptyRuntimeHooks() {
        Gamepad gamepad = new Gamepad(new FakeInput());
        gamepad.a().debounce(java.time.Duration.ofMillis(20)).doubleClick();
        gamepad.leftTrigger(0.5);

        assertTrue(gamepad.hooks().isEmpty());
    }

    @Test
    void toggleResetsOnDisconnectUnlessExplicitlyConfiguredToPersist() {
        FakeInput input = new FakeInput();
        Gamepad gamepad = new Gamepad(input);
        ToggleSignal safe = gamepad.a().toggle();
        ToggleSignal persistent = gamepad.b().toggle().resetOnDisconnect(false);

        ControlSignalTest.sample(safe, 0.0);
        ControlSignalTest.sample(persistent, 0.0);
        input.buttons[1] = true;
        input.buttons[2] = true;
        assertTrue(ControlSignalTest.sample(safe, 0.02));
        assertTrue(ControlSignalTest.sample(persistent, 0.02));

        input.connected = false;
        assertFalse(ControlSignalTest.sample(safe, 0.04));
        assertTrue(ControlSignalTest.sample(persistent, 0.04));
    }

    private static final class FakeInput implements Gamepad.Input {
        private final boolean[] buttons = new boolean[16];
        private boolean connected = true;

        @Override
        public double axis(int axis) {
            return 0.0;
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
}
