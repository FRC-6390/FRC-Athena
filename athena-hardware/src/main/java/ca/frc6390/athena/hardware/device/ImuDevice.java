package ca.frc6390.athena.hardware.device;

import java.util.Locale;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.signal.ImuSource;

/**
 * Reusable IMU declaration.
 */
public record ImuDevice(
        ImuKind kind,
        String bus,
        HardwareAddress connection) implements ImuSource {
    private static final ConcurrentMap<ImuDevice, RuntimeReader> RUNTIME_READERS = new ConcurrentHashMap<>();

    public static ImuDevice of(ImuKind kind, int id) {
        return connected(kind, "rio", new HardwareAddress.Can(id));
    }

    static ImuDevice connected(ImuKind kind, String bus, HardwareAddress connection) {
        return new ImuDevice(kind, bus, connection);
    }

    public ImuDevice {
        Objects.requireNonNull(kind, "kind");
        bus = bus == null || bus.isBlank() ? "rio" : bus;
        Objects.requireNonNull(connection, "connection");
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
        return new ImuDevice(kind, canbus, connection);
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
        return runtimeReader().yawDegrees().getAsDouble();
    }

    /**
     * Returns the latest runtime-refreshed accumulated angle in degrees.
     *
     * @return accumulated angle degrees
     */
    public double angleDegrees() {
        return runtimeReader().angleDegrees().getAsDouble();
    }

    @Override
    public double pitchDegrees() {
        return runtimeReader().pitchDegrees().getAsDouble();
    }

    @Override
    public double rollDegrees() {
        return runtimeReader().rollDegrees().getAsDouble();
    }

    @Override
    public double yawRateDegreesPerSecond() {
        return runtimeReader().yawRateDegreesPerSecond().getAsDouble();
    }

    @Override
    public double linearAccelerationXG() {
        return runtimeReader().linearAccelerationXG().getAsDouble();
    }

    @Override
    public double linearAccelerationYG() {
        return runtimeReader().linearAccelerationYG().getAsDouble();
    }

    @Override
    public double linearAccelerationZG() {
        return runtimeReader().linearAccelerationZG().getAsDouble();
    }

    @Override
    public java.util.List<?> dependencies() {
        return java.util.List.of(this);
    }

    @Override
    public void applyYaw(ActionContext context, double yawDegrees) {
        Objects.requireNonNull(context, "context").imu(this).setYawDegrees(yawDegrees);
    }

    /**
     * Binds runtime-refreshed readings to an IMU declaration.
     *
     * @param device IMU declaration
     * @param yawDegrees yaw reader
     * @param angleDegrees accumulated-angle reader
     */
    public static void bindRuntime(
            ImuDevice device,
            DoubleSupplier yawDegrees,
            DoubleSupplier angleDegrees) {
        DoubleSupplier unsupported = () -> {
            throw new UnsupportedOperationException("IMU reading is not available from this runtime.");
        };
        bindRuntime(device, yawDegrees, unsupported, unsupported, angleDegrees,
                unsupported, unsupported, unsupported, unsupported);
    }

    public static void bindRuntime(
            ImuDevice device,
            DoubleSupplier yawDegrees,
            DoubleSupplier pitchDegrees,
            DoubleSupplier rollDegrees,
            DoubleSupplier angleDegrees,
            DoubleSupplier yawRateDegreesPerSecond,
            DoubleSupplier linearAccelerationXG,
            DoubleSupplier linearAccelerationYG,
            DoubleSupplier linearAccelerationZG) {
        RUNTIME_READERS.put(
                Objects.requireNonNull(device, "device"),
                new RuntimeReader(
                        Objects.requireNonNull(yawDegrees, "yawDegrees"),
                        Objects.requireNonNull(pitchDegrees, "pitchDegrees"),
                        Objects.requireNonNull(rollDegrees, "rollDegrees"),
                        Objects.requireNonNull(angleDegrees, "angleDegrees"),
                        Objects.requireNonNull(yawRateDegreesPerSecond, "yawRateDegreesPerSecond"),
                        Objects.requireNonNull(linearAccelerationXG, "linearAccelerationXG"),
                        Objects.requireNonNull(linearAccelerationYG, "linearAccelerationYG"),
                        Objects.requireNonNull(linearAccelerationZG, "linearAccelerationZG")));
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

    private RuntimeReader runtimeReader() {
        RuntimeReader reader = RUNTIME_READERS.get(this);
        if (reader == null) {
            throw new IllegalStateException("IMU " + defaultName() + " is not runtime-bound.");
        }
        return reader;
    }

    private record RuntimeReader(
            DoubleSupplier yawDegrees,
            DoubleSupplier pitchDegrees,
            DoubleSupplier rollDegrees,
            DoubleSupplier angleDegrees,
            DoubleSupplier yawRateDegreesPerSecond,
            DoubleSupplier linearAccelerationXG,
            DoubleSupplier linearAccelerationYG,
            DoubleSupplier linearAccelerationZG) {
    }
}
