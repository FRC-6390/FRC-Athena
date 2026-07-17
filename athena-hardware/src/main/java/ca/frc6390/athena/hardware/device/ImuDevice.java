package ca.frc6390.athena.hardware.device;

import java.util.Locale;
import java.util.Objects;
import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.RuntimeBindings;
import ca.frc6390.athena.hardware.runtime.RuntimeHardwareAccess;
import ca.frc6390.athena.hardware.runtime.RuntimeScope;
import ca.frc6390.athena.hardware.signal.ImuSource;

/**
 * Reusable IMU declaration.
 */
public record ImuDevice(
        ImuKind kind,
        String bus,
        HardwareAddress connection,
        FailurePolicy failurePolicy) implements ImuSource {
    private static final RuntimeBindings<ImuDevice, ImuHandle> RUNTIMES = new RuntimeBindings<>();

    public ImuDevice(ImuKind kind, String bus, HardwareAddress connection) {
        this(kind, bus, connection, FailurePolicy.DISABLE_MECHANISM);
    }

    public static ImuDevice of(ImuKind kind, int id) {
        return connected(kind, "rio", new HardwareAddress.Can(id));
    }

    static ImuDevice connected(ImuKind kind, String bus, HardwareAddress connection) {
        return new ImuDevice(kind, bus, connection, FailurePolicy.DISABLE_MECHANISM);
    }

    public ImuDevice {
        Objects.requireNonNull(kind, "kind");
        bus = bus == null || bus.isBlank() ? "rio" : bus;
        Objects.requireNonNull(connection, "connection");
        failurePolicy = failurePolicy == null ? FailurePolicy.DISABLE_MECHANISM : failurePolicy;
    }

    public int id() {
        return requireCan().id();
    }

    public String canbus() {
        requireCan();
        return bus;
    }

    public ImuDevice canbus(String canbus) {
        requireCan();
        return new ImuDevice(kind, canbus, connection, failurePolicy);
    }

    /** Selects how Athena responds if this IMU cannot be created or stops responding. */
    public ImuDevice failurePolicy(FailurePolicy policy) {
        return new ImuDevice(kind, bus, connection, Objects.requireNonNull(policy, "policy"));
    }

    public String defaultName() {
        return sanitize(kind.key()) + "_" + sanitize(connection.identity()) + "_" + connection.primaryAddress();
    }

    /**
     * Returns the latest runtime-refreshed yaw in degrees.
     *
     * @return yaw degrees
     */
    public double yawDegrees() {
        return runtimeHandle().yawDegrees();
    }

    /**
     * Returns the latest runtime-refreshed accumulated angle in degrees.
     *
     * @return accumulated angle degrees
     */
    public double angleDegrees() {
        return runtimeHandle().angleDegrees();
    }

    @Override
    public double pitchDegrees() {
        return runtimeHandle().pitchDegrees();
    }

    @Override
    public double rollDegrees() {
        return runtimeHandle().rollDegrees();
    }

    @Override
    public double yawRateDegreesPerSecond() {
        return runtimeHandle().yawRateDegreesPerSecond();
    }

    @Override
    public double linearAccelerationXG() {
        return runtimeHandle().linearAccelerationXG();
    }

    @Override
    public double linearAccelerationYG() {
        return runtimeHandle().linearAccelerationYG();
    }

    @Override
    public double linearAccelerationZG() {
        return runtimeHandle().linearAccelerationZG();
    }

    @Override
    public java.util.List<?> dependencies() {
        return java.util.List.of(this);
    }

    @Override
    public void applyYaw(ActionContext context, double yawDegrees) {
        Objects.requireNonNull(context, "context").imu(this).setYawDegrees(yawDegrees);
    }

    public AutoCloseable bindRuntime(RuntimeScope scope, ImuHandle handle) {
        return RUNTIMES.bind(this, scope, handle);
    }

    private static String sanitize(String key) {
        return key.toLowerCase(Locale.ROOT).replace(':', '_').replace('-', '_');
    }

    private HardwareAddress.Can requireCan() {
        if (connection instanceof HardwareAddress.Can can) {
            return can;
        }
        throw new IllegalStateException("IMU " + kind.key() + " is not connected over CAN.");
    }

    private ImuHandle runtimeHandle() {
        ActionContext current = RuntimeHardwareAccess.current();
        if (current != null) {
            return current.imu(this);
        }
        return RUNTIMES.get(this, "IMU " + defaultName());
    }
}
