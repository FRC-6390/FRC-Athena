package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class GenericControllerTest {
    @Test
    void readsAndCachesRawAxesButtonsAndPovAngles() {
        FakeInput input = new FakeInput();
        GenericController controller = new GenericController(input);
        input.axes[3] = -0.75;
        input.buttons[7] = true;
        input.pov = 135;

        assertEquals(-0.75, controller.axis(3).toSupplier().getAsDouble(), 1.0e-9);
        assertTrue(ControlSignalTest.sample(controller.button(7), 0.0));
        assertTrue(ControlSignalTest.sample(controller.pov(135), 0.0));
        assertEquals(135, controller.pov());
        assertSame(controller.axis(3), controller.axis(3));
        assertSame(controller.button(7), controller.button(7));
        assertSame(controller.pov(135), controller.pov(135));
    }

    @Test
    void validatesWpilibRawIds() {
        GenericController controller = new GenericController(new FakeInput());

        assertThrows(IllegalArgumentException.class, () -> controller.axis(-1));
        assertThrows(IllegalArgumentException.class, () -> controller.button(0));
        assertThrows(IllegalArgumentException.class, () -> controller.pov(-1));
        assertThrows(IllegalArgumentException.class, () -> controller.pov(360));
    }

    @Test
    void participatesInBindingsTelemetryAndDisconnectSafety() {
        FakeInput input = new FakeInput();
        GenericController controller = new GenericController(input);
        ToggleSignal toggle = controller.button(2).toggle();
        toggle.onTrue(() -> {});
        controller.axis(1).deadband(0.1).toSupplier();

        assertFalse(controller.hooks().isEmpty());
        assertTrue(controller.telemetry().containsKey("Connected"));
        assertTrue(controller.telemetry().keySet().stream().anyMatch(key -> key.startsWith("Axes/")));

        ControlSignalTest.sample(toggle, 0.0);
        input.buttons[2] = true;
        assertTrue(ControlSignalTest.sample(toggle, 0.02));
        input.connected = false;
        assertFalse(ControlSignalTest.sample(toggle, 0.04));
    }

    private static final class FakeInput implements GenericController.Input {
        private final double[] axes = new double[12];
        private final boolean[] buttons = new boolean[32];
        private int pov = -1;
        private boolean connected = true;

        @Override
        public double axis(int id) { return axes[id]; }

        @Override
        public boolean button(int id) { return buttons[id]; }

        @Override
        public int pov() { return pov; }

        @Override
        public boolean connected() { return connected; }
    }
}
