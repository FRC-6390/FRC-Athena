package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.HookGroup;
import ca.frc6390.athena.mechanism.core.TelemetrySource;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
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
public class Gamepad extends ControlOwner implements HookGroup, TelemetrySource {
    private static final double DIGITAL_TRIGGER_THRESHOLD = 0.5;

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
    private final ButtonSignal l2;
    private final ButtonSignal r2;
    private final ButtonSignal povUp;
    private final ButtonSignal povRight;
    private final ButtonSignal povDown;
    private final ButtonSignal povLeft;
    private final List<ControlSignal> signals;
    private final Map<String, AxisSignal> registeredAxes;

    Gamepad(XboxController controller) {
        this(new XboxInput(controller), Layout.XBOX);
    }

    Gamepad(Input controller) {
        this(controller, Layout.XBOX);
    }

    Gamepad(Input controller, Layout layout) {
        this.controller = Objects.requireNonNull(controller, "controller");
        Objects.requireNonNull(layout, "layout");
        signals = new ArrayList<>();
        registeredAxes = new LinkedHashMap<>();
        leftX = axis("leftX", layout.leftX());
        leftY = axis("leftY", layout.leftY());
        rightX = axis("rightX", layout.rightX());
        rightY = axis("rightY", layout.rightY());
        leftTrigger = axis("leftTrigger", layout.leftTrigger());
        rightTrigger = axis("rightTrigger", layout.rightTrigger());
        a = button("a", layout.faceBottom());
        b = button("b", layout.faceRight());
        x = button("x", layout.faceLeft());
        y = button("y", layout.faceTop());
        leftBumper = button("leftBumper", layout.leftBumper());
        rightBumper = button("rightBumper", layout.rightBumper());
        back = button("back", layout.back());
        start = button("start", layout.start());
        leftStick = button("leftStick", layout.leftStick());
        rightStick = button("rightStick", layout.rightStick());
        l2 = buttonFrom(leftTrigger.above(DIGITAL_TRIGGER_THRESHOLD));
        r2 = buttonFrom(rightTrigger.above(DIGITAL_TRIGGER_THRESHOLD));
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

    /** PlayStation-style alias for the bottom face button (Xbox A). */
    public ButtonSignal cross() { return a; }
    /** PlayStation-style alias for the right face button (Xbox B). */
    public ButtonSignal circle() { return b; }
    /** PlayStation-style alias for the left face button (Xbox X). */
    public ButtonSignal square() { return x; }
    /** PlayStation-style alias for the top face button (Xbox Y). */
    public ButtonSignal triangle() { return y; }
    public ButtonSignal l1() { return leftBumper; }
    public ButtonSignal r1() { return rightBumper; }
    public AxisSignal l2Axis() { return leftTrigger; }
    public AxisSignal r2Axis() { return rightTrigger; }
    /** Digital view of the left trigger, active above 50%. */
    public ButtonSignal l2() { return l2; }
    /** Digital view of the right trigger, active above 50%. */
    public ButtonSignal r2() { return r2; }
    public ButtonSignal share() { return back; }
    public ButtonSignal options() { return start; }
    public ButtonSignal l3() { return leftStick; }
    public ButtonSignal r3() { return rightStick; }

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

    @Override
    public Map<String, TelemetryValue> telemetry() {
        Map<String, TelemetryValue> values = new LinkedHashMap<>();
        values.put("Connected", TelemetryValue.bool(controller::connected));
        registeredAxes.forEach((axisName, axis) -> axis.telemetry().forEach(
                (name, value) -> values.put("Axes/" + axisName + "/" + name, value)));
        return Map.copyOf(values);
    }

    void register(ControlSignal signal) {
        signals.add(Objects.requireNonNull(signal, "signal"));
    }

    void register(AxisSignal axis) {
        Objects.requireNonNull(axis, "axis");
        if (registeredAxes.containsValue(axis)) return;

        String baseName = axis.name();
        String registeredName = baseName;
        for (int suffix = 2; registeredAxes.containsKey(registeredName); suffix++) {
            registeredName = baseName + suffix;
        }
        registeredAxes.put(registeredName, axis);
    }

    boolean connected() {
        return controller.connected();
    }

    private AxisSignal axis(String name, int axis) {
        return new AxisSignal(this, name, () -> controller.axis(axis));
    }

    final ButtonSignal button(String name, int button) {
        return signal(name, () -> controller.button(button));
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

    record Layout(
            int leftX,
            int leftY,
            int rightX,
            int rightY,
            int leftTrigger,
            int rightTrigger,
            int faceBottom,
            int faceRight,
            int faceLeft,
            int faceTop,
            int leftBumper,
            int rightBumper,
            int back,
            int start,
            int leftStick,
            int rightStick) {
        private static final Layout XBOX = new Layout(
                XboxController.Axis.kLeftX.value,
                XboxController.Axis.kLeftY.value,
                XboxController.Axis.kRightX.value,
                XboxController.Axis.kRightY.value,
                XboxController.Axis.kLeftTrigger.value,
                XboxController.Axis.kRightTrigger.value,
                XboxController.Button.kA.value,
                XboxController.Button.kB.value,
                XboxController.Button.kX.value,
                XboxController.Button.kY.value,
                XboxController.Button.kLeftBumper.value,
                XboxController.Button.kRightBumper.value,
                XboxController.Button.kBack.value,
                XboxController.Button.kStart.value,
                XboxController.Button.kLeftStick.value,
                XboxController.Button.kRightStick.value);
    }
}
