package ca.frc6390.athena.wpilib.controls;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotBase;
import java.util.Objects;

/**
 * A PlayStation gamepad with native button names and the standard {@link Gamepad} aliases.
 *
 * <p>The inherited A/B/X/Y signals refer to the equivalent physical positions: cross, circle,
 * square, and triangle respectively. Back/start, bumpers, and stick buttons similarly map to
 * share/options, L1/R1, and L3/R3.
 */
public final class PlayStationGamepad extends Gamepad {
    private static final double WINDOWS_SIM_CENTER_TOLERANCE = 0.01;

    private static final Layout PLAYSTATION = new Layout(
            PS4Controller.Axis.kLeftX.value,
            PS4Controller.Axis.kLeftY.value,
            PS4Controller.Axis.kRightX.value,
            PS4Controller.Axis.kRightY.value,
            PS4Controller.Axis.kL2.value,
            PS4Controller.Axis.kR2.value,
            PS4Controller.Button.kCross.value,
            PS4Controller.Button.kCircle.value,
            PS4Controller.Button.kSquare.value,
            PS4Controller.Button.kTriangle.value,
            PS4Controller.Button.kL1.value,
            PS4Controller.Button.kR1.value,
            PS4Controller.Button.kShare.value,
            PS4Controller.Button.kOptions.value,
            PS4Controller.Button.kL3.value,
            PS4Controller.Button.kR3.value);

    private final ButtonSignal l2;
    private final ButtonSignal r2;
    private final ButtonSignal ps;
    private final ButtonSignal touchpad;

    PlayStationGamepad(PS4Controller controller) {
        this(new PlayStationInput(controller), usesWindowsSimulationLayout());
    }

    PlayStationGamepad(Input controller) {
        this(controller, false);
    }

    PlayStationGamepad(Input controller, boolean windowsSimulation) {
        super(windowsSimulation ? new WindowsSimulationInput(controller) : controller, PLAYSTATION);
        l2 = button("l2", PS4Controller.Button.kL2.value);
        r2 = button("r2", PS4Controller.Button.kR2.value);
        ps = button("ps", PS4Controller.Button.kPS.value);
        touchpad = button("touchpad", PS4Controller.Button.kTouchpad.value);
    }

    @Override
    public ButtonSignal l2() { return l2; }
    @Override
    public ButtonSignal r2() { return r2; }
    public ButtonSignal ps() { return ps; }
    public ButtonSignal touchpad() { return touchpad; }

    private static boolean usesWindowsSimulationLayout() {
        return RobotBase.isSimulation()
                && System.getProperty("os.name", "").toLowerCase(java.util.Locale.ROOT).startsWith("windows");
    }

    /** Adapts SDL's Windows DualShock axis order to WPILib's PS4 axis contract. */
    private static final class WindowsSimulationInput implements Input {
        private final Input controller;

        private WindowsSimulationInput(Input controller) {
            this.controller = Objects.requireNonNull(controller, "controller");
        }

        @Override
        public double axis(int axis) {
            if (axis == PS4Controller.Axis.kRightY.value) {
                return zeroCentered(controller.axis(3));
            }
            if (axis == PS4Controller.Axis.kL2.value) {
                return normalizeTrigger(controller.axis(4));
            }
            if (axis == PS4Controller.Axis.kR2.value) {
                return normalizeTrigger(controller.axis(5));
            }
            return zeroCentered(controller.axis(axis));
        }

        @Override
        public boolean button(int button) {
            return controller.button(button);
        }

        @Override
        public int pov() {
            return controller.pov();
        }

        @Override
        public boolean connected() {
            return controller.connected();
        }

        private static double normalizeTrigger(double value) {
            return (value + 1.0) / 2.0;
        }

        private static double zeroCentered(double value) {
            return Math.abs(value) <= WINDOWS_SIM_CENTER_TOLERANCE ? 0.0 : value;
        }
    }

    private static final class PlayStationInput implements Input {
        private final PS4Controller controller;

        private PlayStationInput(PS4Controller controller) {
            this.controller = Objects.requireNonNull(controller, "controller");
        }

        @Override
        public double axis(int axis) {
            return controller.getRawAxis(axis);
        }

        @Override
        public boolean button(int button) {
            return controller.getRawButton(button);
        }

        @Override
        public int pov() {
            return controller.getPOV();
        }

        @Override
        public boolean connected() {
            return controller.isConnected();
        }
    }
}
