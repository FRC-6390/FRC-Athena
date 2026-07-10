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
    private final XboxController controller;
    private final AxisSignal leftX;
    private final AxisSignal leftY;
    private final AxisSignal rightX;
    private final AxisSignal rightY;
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
    private final List<ButtonSignal> buttons;

    Gamepad(XboxController controller) {
        this.controller = Objects.requireNonNull(controller, "controller");
        buttons = new ArrayList<>();
        leftX = axis(XboxController.Axis.kLeftX);
        leftY = axis(XboxController.Axis.kLeftY);
        rightX = axis(XboxController.Axis.kRightX);
        rightY = axis(XboxController.Axis.kRightY);
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
        return signal("leftTrigger", () -> controller.getLeftTriggerAxis() > threshold);
    }

    public ButtonSignal rightTrigger(double threshold) {
        return signal("rightTrigger", () -> controller.getRightTriggerAxis() > threshold);
    }

    public ButtonSignal signal(String name, BooleanSupplier source) {
        ButtonSignal signal = new ButtonSignal(name, source);
        buttons.add(signal);
        return signal;
    }

    @Override
    public Map<String, HookBinding> hooks() {
        Map<String, HookBinding> hooks = new LinkedHashMap<>();
        for (int index = 0; index < buttons.size(); index++) {
            ButtonSignal button = buttons.get(index);
            if (!button.binding().actions().isEmpty()) {
                hooks.put(button.name() + "." + index, button.binding());
            }
        }
        return Map.copyOf(hooks);
    }

    private AxisSignal axis(XboxController.Axis axis) {
        return new AxisSignal(() -> controller.getRawAxis(axis.value));
    }

    private ButtonSignal button(String name, XboxController.Button button) {
        return signal(name, () -> controller.getRawButton(button.value));
    }

    private ButtonSignal pov(String name, int angle) {
        return signal(name, () -> controller.getPOV() == angle);
    }
}
