package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.IdentityHashMap;
import java.util.Map;

/** Scheduler-owned runtime overrides for declarations whose deployment config stays immutable. */
final class RuntimeOverrides {
    private final Map<MotorDevice, Boolean> motors = new IdentityHashMap<>();
    private final Map<ControlBinding, Boolean> controls = new IdentityHashMap<>();
    private final Map<MotorDevice, TelemetryValue> motorTelemetry = new IdentityHashMap<>();
    private final Map<ControlBinding, TelemetryValue> controlTelemetry = new IdentityHashMap<>();

    boolean disabled(MotorDevice motor) { return motors.getOrDefault(motor, motor.isDisabled()); }
    boolean disabled(ControlBinding control) { return controls.getOrDefault(control, control.isDisabled()); }
    void disabled(MotorDevice motor, boolean value) { motors.put(motor, value); }
    void disabled(ControlBinding control, boolean value) { controls.put(control, value); }

    TelemetryValue motorDisabled(MotorDevice motor) {
        return motorTelemetry.computeIfAbsent(motor, declaration -> TelemetryValue.writableBoolean(
                () -> disabled(declaration), value -> disabled(declaration, value)));
    }

    TelemetryValue controlDisabled(ControlBinding control) {
        return controlTelemetry.computeIfAbsent(control, declaration -> TelemetryValue.writableBoolean(
                () -> disabled(declaration), value -> disabled(declaration, value)));
    }
}
