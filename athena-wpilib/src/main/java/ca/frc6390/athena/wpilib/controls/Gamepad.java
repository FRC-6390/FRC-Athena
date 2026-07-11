package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.HookGroup;
import edu.wpi.first.wpilibj.XboxController;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * A gamepad declaration exposing processed axes and directly bindable buttons.
 */
public final class Gamepad implements HookGroup {
    interface Input {
        double axis(int axis);

        boolean button(int button);

        int pov();

        boolean connected();
    }

    private final Input controller;
    private final AxisSignal leftX;
    private final AxisSignal leftY;
    private final AxisSignal rightX;
    private final AxisSignal rightY;
    private final AxisSignal leftTrigger;
    private final AxisSignal rightTrigger;
    private final ButtonSignal a;
    private final ButtonSignal b;
    private final ButtonSignal x;
    private final ButtonSignal y;
    private final ButtonSignal leftBumper;
    private final ButtonSignal rightBumper;
    private final ButtonSignal back;
    private final ButtonSignal start;
    private final ButtonSignal leftStick;
    private final ButtonSignal rightStick;
    private final ButtonSignal povUp;
    private final ButtonSignal povRight;
    private final ButtonSignal povDown;
    private final ButtonSignal povLeft;
    private final List<ControlSignal> signals;

    Gamepad(XboxController controller) {
        this(new XboxInput(controller));
    }

    Gamepad(Input controller) {
        this.controller = Objects.requireNonNull(controller, "controller");
        signals = new ArrayList<>();
        leftX = axis("leftX", XboxController.Axis.kLeftX);
        leftY = axis("leftY", XboxController.Axis.kLeftY);
        rightX = axis("rightX", XboxController.Axis.kRightX);
        rightY = axis("rightY", XboxController.Axis.kRightY);
        leftTrigger = axis("leftTrigger", XboxController.Axis.kLeftTrigger);
        rightTrigger = axis("rightTrigger", XboxController.Axis.kRightTrigger);
        a = button("a", XboxController.Button.kA);
        b = button("b", XboxController.Button.kB);
        x = button("x", XboxController.Button.kX);
        y = button("y", XboxController.Button.kY);
        leftBumper = button("leftBumper", XboxController.Button.kLeftBumper);
        rightBumper = button("rightBumper", XboxController.Button.kRightBumper);
        back = button("back", XboxController.Button.kBack);
        start = button("start", XboxController.Button.kStart);
        leftStick = button("leftStick", XboxController.Button.kLeftStick);
        rightStick = button("rightStick", XboxController.Button.kRightStick);
        povUp = pov("povUp", 0);
        povRight = pov("povRight", 90);
        povDown = pov("povDown", 180);
        povLeft = pov("povLeft", 270);
    }

    public AxisSignal leftX() { return leftX; }
    public AxisSignal leftY() { return leftY; }
    public AxisSignal rightX() { return rightX; }
    public AxisSignal rightY() { return rightY; }
    public AxisSignal leftTrigger() { return leftTrigger; }
    public AxisSignal rightTrigger() { return rightTrigger; }
    public ButtonSignal a() { return a; }
    public ButtonSignal b() { return b; }
    public ButtonSignal x() { return x; }
    public ButtonSignal y() { return y; }
    public ButtonSignal leftBumper() { return leftBumper; }
    public ButtonSignal rightBumper() { return rightBumper; }
    public ButtonSignal back() { return back; }
    public ButtonSignal start() { return start; }
    public ButtonSignal leftStick() { return leftStick; }
    public ButtonSignal rightStick() { return rightStick; }
    public ButtonSignal povUp() { return povUp; }
    public ButtonSignal povRight() { return povRight; }
    public ButtonSignal povDown() { return povDown; }
    public ButtonSignal povLeft() { return povLeft; }

    public ButtonSignal leftTrigger(double threshold) {
        return buttonFrom(leftTrigger.above(threshold));
    }

    public ButtonSignal rightTrigger(double threshold) {
        return buttonFrom(rightTrigger.above(threshold));
    }

    public ButtonSignal signal(String name, BooleanSupplier source) {
        Objects.requireNonNull(source, "source");
        return new ButtonSignal(this, name, context -> source.getAsBoolean());
    }

    @Override
    public Map<String, HookBinding> hooks() {
        Map<String, HookBinding> hooks = new LinkedHashMap<>();
        for (int index = 0; index < signals.size(); index++) {
            ControlSignal signal = signals.get(index);
            if (!signal.binding().actions().isEmpty()) {
                hooks.put(signal.name() + "." + index, signal.binding());
            }
        }
        return Map.copyOf(hooks);
    }

    void register(ControlSignal signal) {
        signals.add(Objects.requireNonNull(signal, "signal"));
    }

    boolean connected() {
        return controller.connected();
    }

    private AxisSignal axis(String name, XboxController.Axis axis) {
        return new AxisSignal(this, name, () -> controller.axis(axis.value));
    }

    private ButtonSignal button(String name, XboxController.Button button) {
        return signal(name, () -> controller.button(button.value));
    }

    private ButtonSignal pov(String name, int angle) {
        return signal(name, () -> controller.pov() == angle);
    }

    private ButtonSignal buttonFrom(ControlSignal signal) {
        return new ButtonSignal(this, signal.name(), signal::sample);
    }

    private static final class XboxInput implements Input {
        private final XboxController controller;

        private XboxInput(XboxController controller) {
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
