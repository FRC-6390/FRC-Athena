package ca.frc6390.athena.wpilib.controls;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import edu.wpi.first.wpilibj.XboxController;
import org.junit.jupiter.api.Test;

class XboxPlayStationAliasesTest {
    @Test
    void exposesPlayStationNamesForEquivalentXboxButtons() {
        FakeInput input = new FakeInput();
        Gamepad gamepad = new Gamepad(input);

        assertSame(gamepad.a(), gamepad.cross());
        assertSame(gamepad.b(), gamepad.circle());
        assertSame(gamepad.x(), gamepad.square());
        assertSame(gamepad.y(), gamepad.triangle());
        assertSame(gamepad.leftBumper(), gamepad.l1());
        assertSame(gamepad.rightBumper(), gamepad.r1());
        assertSame(gamepad.back(), gamepad.share());
        assertSame(gamepad.start(), gamepad.options());
        assertSame(gamepad.leftStick(), gamepad.l3());
        assertSame(gamepad.rightStick(), gamepad.r3());
    }

    @Test
    void exposesXboxTriggersAsPlayStationAnalogAndDigitalControls() {
        FakeInput input = new FakeInput();
        Gamepad gamepad = new Gamepad(input);

        assertSame(gamepad.leftTrigger(), gamepad.l2Axis());
        assertSame(gamepad.rightTrigger(), gamepad.r2Axis());
        input.axes[XboxController.Axis.kLeftTrigger.value] = 0.49;
        assertFalse(ControlSignalTest.sample(gamepad.l2(), 0.0));
        input.axes[XboxController.Axis.kLeftTrigger.value] = 0.51;
        assertTrue(ControlSignalTest.sample(gamepad.l2(), 0.02));
    }

    private static final class FakeInput implements Gamepad.Input {
        private final double[] axes = new double[8];

        @Override
        public double axis(int axis) {
            return axes[axis];
        }

        @Override
        public boolean button(int button) {
            return false;
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
