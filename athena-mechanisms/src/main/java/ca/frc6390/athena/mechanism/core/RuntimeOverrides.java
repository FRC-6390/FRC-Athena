package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.motion.MotionProfile;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.hardware.device.Range;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.Set;

/** Scheduler-owned runtime overrides for declarations whose deployment config stays immutable. */
final class RuntimeOverrides {
    private final Map<MotorDevice, Boolean> motors = new IdentityHashMap<>();
    private final Map<ControlBinding, Boolean> controls = new IdentityHashMap<>();
    private final Map<MotorDevice, TelemetryValue> motorTelemetry = new IdentityHashMap<>();
    private final Map<ControlBinding, TelemetryValue> controlTelemetry = new IdentityHashMap<>();
    private final Map<ControlBinding, Double> controlTestPercent = new IdentityHashMap<>();
    private final Map<ControlBinding, ControlTuning> controlTunings = new IdentityHashMap<>();
    private final Set<ControlBinding> controlProfileOverrides = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    private final Map<MotorDevice, MotorRuntimeConfig> motorConfigs = new IdentityHashMap<>();
    private final Map<MotorDevice, String> motorStatus = new IdentityHashMap<>();
    private final Map<Object, String> setupStatus = new IdentityHashMap<>();
    private final Map<EncoderDevice, Double> requestedEncoderPositions = new IdentityHashMap<>();
    private final Map<ImuDevice, Double> requestedImuYaw = new IdentityHashMap<>();
    private ActionContext actionContext = ActionContext.empty();

    void actionContext(ActionContext actionContext) { this.actionContext = actionContext; }

    boolean disabled(MotorDevice motor) { return motors.getOrDefault(motor, motor.isDisabled()); }
    boolean disabled(ControlBinding control) { return controls.getOrDefault(control, control.isDisabled()); }
    void disabled(MotorDevice motor, boolean value) { motors.put(motor, value); }
    void disabled(ControlBinding control, boolean value) { controls.put(control, value); }
    void restore(MotorDevice motor) {
        motors.remove(motor);
        motorConfigs.remove(motor);
        apply(motor, MotorRuntimeConfig.declared(motor));
    }

    MotorRuntimeConfig config(MotorDevice motor) {
        return motorConfigs.getOrDefault(motor, MotorRuntimeConfig.declared(motor));
    }

    boolean supportsMotorConfig(MotorDevice motor) {
        try { return actionContext.motor(motor).supportsRuntimeConfiguration(); }
        catch (RuntimeException exception) { return false; }
    }

    String motorStatus(MotorDevice motor) { return motorStatus.getOrDefault(motor, "Declared"); }

    boolean supportsEncoderPosition(EncoderDevice encoder) {
        try { return actionContext.encoder(encoder).supportsPositionSetting(); }
        catch (RuntimeException exception) { return false; }
    }

    void encoderPosition(EncoderDevice encoder, double position) {
        try {
            actionContext.encoder(encoder).setPositionRotations(encoder.rotationsFromPosition(position));
            setupStatus.put(encoder, "Applied");
        } catch (RuntimeException exception) {
            setupStatus.put(encoder, "Error: " + exception.getMessage());
        }
    }

    double requestedEncoderPosition(EncoderDevice encoder) {
        return requestedEncoderPositions.computeIfAbsent(encoder, ignored -> safeRead(encoder::position));
    }

    void requestedEncoderPosition(EncoderDevice encoder, double position) {
        if (Double.isFinite(position)) requestedEncoderPositions.put(encoder, position);
    }

    void imuYaw(ImuDevice imu, double yawDegrees) {
        try {
            actionContext.imu(imu).setYawDegrees(yawDegrees);
            setupStatus.put(imu, "Applied");
        } catch (RuntimeException exception) {
            setupStatus.put(imu, "Error: " + exception.getMessage());
        }
    }

    double requestedImuYaw(ImuDevice imu) {
        return requestedImuYaw.computeIfAbsent(imu, ignored -> safeRead(imu::yawDegrees));
    }

    void requestedImuYaw(ImuDevice imu, double yawDegrees) {
        if (Double.isFinite(yawDegrees)) requestedImuYaw.put(imu, yawDegrees);
    }

    private static double safeRead(java.util.function.DoubleSupplier reader) {
        try { return reader.getAsDouble(); }
        catch (RuntimeException exception) { return 0.0; }
    }

    String setupStatus(Object device) { return setupStatus.getOrDefault(device, "Ready"); }

    void neutralMode(MotorDevice motor, String value) {
        update(motor, new MotorRuntimeConfig(MotorNeutralMode.valueOf(value.trim().toUpperCase(java.util.Locale.ROOT)),
                config(motor).inverted(), config(motor).supplyCurrentLimitAmps(), config(motor).statorCurrentLimitAmps()));
    }

    void inverted(MotorDevice motor, boolean value) {
        MotorRuntimeConfig current = config(motor);
        update(motor, new MotorRuntimeConfig(current.neutralMode(), value,
                current.supplyCurrentLimitAmps(), current.statorCurrentLimitAmps()));
    }

    void supplyLimit(MotorDevice motor, double value) {
        MotorRuntimeConfig current = config(motor);
        update(motor, new MotorRuntimeConfig(current.neutralMode(), current.inverted(),
                nonNegativeAmps(value), current.statorCurrentLimitAmps()));
    }

    void statorLimit(MotorDevice motor, double value) {
        MotorRuntimeConfig current = config(motor);
        update(motor, new MotorRuntimeConfig(current.neutralMode(), current.inverted(),
                current.supplyCurrentLimitAmps(), nonNegativeAmps(value)));
    }

    private static int nonNegativeAmps(double value) {
        if (!Double.isFinite(value) || value < 0 || value > Integer.MAX_VALUE) {
            throw new IllegalArgumentException("Current limit must be a non-negative finite number.");
        }
        return (int) Math.round(value);
    }

    private void update(MotorDevice motor, MotorRuntimeConfig config) {
        if (config.equals(config(motor))) return;
        motorConfigs.put(motor, config);
        apply(motor, config);
    }

    private void apply(MotorDevice motor, MotorRuntimeConfig config) {
        try {
            MotorHandle handle = actionContext.motor(motor);
            if (!handle.supportsRuntimeConfiguration()) {
                motorStatus.put(motor, "Unsupported");
                return;
            }
            handle.applyRuntimeConfiguration(config);
            motorStatus.put(motor, "Applied");
        } catch (RuntimeException exception) {
            motorStatus.put(motor, "Error: " + exception.getMessage());
        }
    }

    TelemetryValue motorDisabled(MotorDevice motor) {
        return motorTelemetry.computeIfAbsent(motor, declaration -> TelemetryValue.writableBoolean(
                () -> disabled(declaration), value -> disabled(declaration, value)));
    }

    TelemetryValue controlDisabled(ControlBinding control) {
        return controlTelemetry.computeIfAbsent(control, declaration -> TelemetryValue.writableBoolean(
                () -> disabled(declaration), value -> disabled(declaration, value)));
    }

    double testPercent(ControlBinding control) { return controlTestPercent.getOrDefault(control, 0.0); }
    void testPercent(ControlBinding control, double value) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException("Test percent must be finite.");
        controlTestPercent.put(control, Math.max(-1.0, Math.min(1.0, value)));
    }

    ControlTuning tuning(ControlBinding control) {
        return controlTunings.getOrDefault(control, ControlTuning.declared(control));
    }

    boolean hasTuning(ControlBinding control) {
        return controlTunings.containsKey(control);
    }

    boolean hasProfile(ControlBinding control) {
        return control.profile() != null
                || control.mode() == ControlMode.POSITION
                        && Constraints.motionProfile(control.constraints()) != null
                || controlProfileOverrides.contains(control);
    }

    MotionProfile profile(ControlBinding control) {
        ControlTuning tuning = tuning(control);
        return new MotionProfile(tuning.maxVelocity(), tuning.maxAcceleration());
    }

    void minimumPosition(ControlBinding control, double value) {
        requireFinite(value, "Minimum position");
        ControlTuning current = tuning(control);
        if (value >= current.maximumPosition()) throw new IllegalArgumentException("Minimum position must be below maximum position.");
        controlTunings.put(control, current.withPositions(value, current.maximumPosition()));
    }

    void maximumPosition(ControlBinding control, double value) {
        requireFinite(value, "Maximum position");
        ControlTuning current = tuning(control);
        if (value <= current.minimumPosition()) throw new IllegalArgumentException("Maximum position must be above minimum position.");
        controlTunings.put(control, current.withPositions(current.minimumPosition(), value));
    }

    void maximumVelocity(ControlBinding control, double value) {
        requirePositive(value, "Maximum velocity");
        ControlTuning current = tuning(control);
        controlTunings.put(control, current.withMotion(value, current.maxAcceleration()));
        controlProfileOverrides.add(control);
    }

    void maximumAcceleration(ControlBinding control, double value) {
        requirePositive(value, "Maximum acceleration");
        ControlTuning current = tuning(control);
        controlTunings.put(control, current.withMotion(current.maxVelocity(), value));
        controlProfileOverrides.add(control);
    }

    void restore(ControlBinding control) {
        controlTunings.remove(control);
        controlProfileOverrides.remove(control);
        controlTestPercent.remove(control);
    }

    private static void requireFinite(double value, String name) {
        if (!Double.isFinite(value)) throw new IllegalArgumentException(name + " must be finite.");
    }

    private static void requirePositive(double value, String name) {
        requireFinite(value, name);
        if (value <= 0.0) throw new IllegalArgumentException(name + " must be positive.");
    }

    record ControlTuning(
            double minimumPosition,
            double maximumPosition,
            double maxVelocity,
            double maxAcceleration) {
        static ControlTuning declared(ControlBinding control) {
            Range range = Constraints.positionRange(control.constraints());
            MotionProfile profile = control.profile() == null
                    ? Constraints.motionProfile(control.constraints())
                    : control.profile();
            return new ControlTuning(
                    range.minimum(),
                    range.maximum(),
                    profile == null ? Double.MAX_VALUE : profile.maxVelocity(),
                    profile == null ? Double.MAX_VALUE : profile.maxAcceleration());
        }
        ControlTuning withPositions(double minimum, double maximum) {
            return new ControlTuning(minimum, maximum, maxVelocity, maxAcceleration);
        }
        ControlTuning withMotion(double velocity, double acceleration) {
            return new ControlTuning(minimumPosition, maximumPosition, velocity, acceleration);
        }
    }
}
