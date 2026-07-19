package ca.frc6390.athena.wpilib.controls;

import ca.frc6390.athena.mechanism.core.HookBinding;
import ca.frc6390.athena.mechanism.core.HookGroup;
import ca.frc6390.athena.mechanism.core.TelemetrySource;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import edu.wpi.first.wpilibj.GenericHID;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/**
 * A controller with raw axis and button IDs for custom or otherwise unmapped HID devices.
 *
 * <p>Axis IDs are zero-based, while WPILib button IDs are one-based. Signals are cached by ID,
 * so asking for the same input repeatedly returns the same declaration.
 */
public final class GenericController extends ControlOwner implements HookGroup, TelemetrySource {
    interface Input {
        double axis(int id);
        boolean button(int id);
        int pov();
        boolean connected();
    }

    private final Input controller;
    private final Map<Integer, AxisSignal> axes = new LinkedHashMap<>();
    private final Map<Integer, ButtonSignal> buttons = new LinkedHashMap<>();
    private final Map<Integer, ButtonSignal> povButtons = new LinkedHashMap<>();
    private final List<ControlSignal> signals = new ArrayList<>();
    private final Map<String, AxisSignal> registeredAxes = new LinkedHashMap<>();

    GenericController(GenericHID controller) {
        this(new GenericInput(controller));
    }

    GenericController(Input controller) {
        this.controller = Objects.requireNonNull(controller, "controller");
    }

    /** Returns the processed axis declaration for a zero-based raw axis ID. */
    public AxisSignal axis(int id) {
        requireAxisId(id);
        return axes.computeIfAbsent(id,
                axisId -> new AxisSignal(this, "axis[" + axisId + "]", () -> controller.axis(axisId)));
    }

    /** Returns the bindable button declaration for a one-based raw button ID. */
    public ButtonSignal button(int id) {
        requireButtonId(id);
        return buttons.computeIfAbsent(id,
                buttonId -> new ButtonSignal(this, "button[" + buttonId + "]",
                        context -> controller.button(buttonId)));
    }

    /** Returns a bindable signal for a POV angle in degrees. */
    public ButtonSignal pov(int angle) {
        if (angle < 0 || angle >= 360) {
            throw new IllegalArgumentException("POV angle must be between 0 and 359 degrees");
        }
        return povButtons.computeIfAbsent(angle,
                povAngle -> new ButtonSignal(this, "pov[" + povAngle + "]",
                        context -> controller.pov() == povAngle));
    }

    /** Returns the current raw POV angle, or -1 when it is not pressed. */
    public int pov() {
        return controller.pov();
    }

    /** Reports whether Driver Station currently sees the HID device. */
    @Override
    public boolean connected() {
        return controller.connected();
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

    @Override
    void register(ControlSignal signal) {
        signals.add(Objects.requireNonNull(signal, "signal"));
    }

    @Override
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

    private static void requireAxisId(int id) {
        if (id < 0) throw new IllegalArgumentException("axis ID must be zero or greater");
    }

    private static void requireButtonId(int id) {
        if (id < 1) throw new IllegalArgumentException("button ID must be one or greater");
    }

    private static final class GenericInput implements Input {
        private final GenericHID controller;

        private GenericInput(GenericHID controller) {
            this.controller = Objects.requireNonNull(controller, "controller");
        }

        @Override
        public double axis(int id) { return controller.getRawAxis(id); }

        @Override
        public boolean button(int id) { return controller.getRawButton(id); }

        @Override
        public int pov() { return controller.getPOV(); }

        @Override
        public boolean connected() { return controller.isConnected(); }
    }
}
