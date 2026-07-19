package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.PS4Controller;
import org.junit.jupiter.api.Test;

class PlayStationGamepadTest {
    @Test
    void mapsPlayStationAxesAndFaceButtons() {
        FakeInput input = new FakeInput();
        PlayStationGamepad gamepad = new PlayStationGamepad(input);
        input.axes[PS4Controller.Axis.kLeftX.value] = 0.25;
        input.axes[PS4Controller.Axis.kR2.value] = 0.75;
        input.buttons[PS4Controller.Button.kCross.value] = true;

        assertEquals(0.25, gamepad.leftX().toSupplier().getAsDouble(), 1.0e-9);
        assertEquals(0.75, gamepad.r2Axis().toSupplier().getAsDouble(), 1.0e-9);
        assertTrue(ControlSignalTest.sample(gamepad.cross(), 0.0));
        assertSame(gamepad.cross(), gamepad.a());
        assertSame(gamepad.circle(), gamepad.b());
        assertSame(gamepad.square(), gamepad.x());
        assertSame(gamepad.triangle(), gamepad.y());
    }

    @Test
    void exposesPlayStationSpecificDigitalButtons() {
        FakeInput input = new FakeInput();
        PlayStationGamepad gamepad = new PlayStationGamepad(input);
        input.buttons[PS4Controller.Button.kL2.value] = true;
        input.buttons[PS4Controller.Button.kPS.value] = true;
        input.buttons[PS4Controller.Button.kTouchpad.value] = true;

        assertTrue(ControlSignalTest.sample(gamepad.l2(), 0.0));
        assertTrue(ControlSignalTest.sample(gamepad.ps(), 0.0));
        assertTrue(ControlSignalTest.sample(gamepad.touchpad(), 0.0));
    }

    @Test
    void remapsWindowsSimulationAxesAndNormalizesTriggers() {
        FakeInput input = new FakeInput();
        PlayStationGamepad gamepad = new PlayStationGamepad(input, true);
        input.axes[2] = 1.0 / 256.0;
        input.axes[3] = -1.0 / 256.0;
        input.axes[4] = -1.0;
        input.axes[5] = 1.0;

        assertEquals(0.0, gamepad.rightX().toSupplier().getAsDouble(), 1.0e-9);
        assertEquals(0.0, gamepad.rightY().toSupplier().getAsDouble(), 1.0e-9);
        input.axes[2] = 0.2;
        input.axes[3] = -0.3;
        assertEquals(0.2, gamepad.rightX().toSupplier().getAsDouble(), 1.0e-9);
        assertEquals(-0.3, gamepad.rightY().toSupplier().getAsDouble(), 1.0e-9);
        assertEquals(0.0, gamepad.l2Axis().toSupplier().getAsDouble(), 1.0e-9);
        assertEquals(1.0, gamepad.r2Axis().toSupplier().getAsDouble(), 1.0e-9);
    }

    private static final class FakeInput implements Gamepad.Input {
        private final double[] axes = new double[8];
        private final boolean[] buttons = new boolean[20];

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
            return true;
        }
    }
}
