package ca.frc6390.athena.hardware.device;

import java.util.Locale;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.ConcurrentMap;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.api.hardware.ImuKind;

/**
 * Reusable IMU declaration.
 */
public record ImuDevice(
        ImuKind kind,
        String bus,
        HardwarePort port) {
    private static final ConcurrentMap<ImuDevice, RuntimeReader> RUNTIME_READERS = new ConcurrentHashMap<>();

    public static ImuDevice of(ImuKind kind, int id) {
        return connected(kind, "rio", HardwarePort.can(id));
    }

    static ImuDevice connected(ImuKind kind, String bus, HardwarePort port) {
        return new ImuDevice(kind, bus, port);
    }

    public ImuDevice {
        Objects.requireNonNull(kind, "kind");
        bus = bus == null || bus.isBlank() ? "rio" : bus;
        Objects.requireNonNull(port, "port");
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
        return new ImuDevice(kind, canbus, port);
    }

    public String defaultName() {
        return sanitize(kind.key()) + "_" + sanitize(port.identity()) + "_" + port.primaryAddress();
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
        RUNTIME_READERS.put(
                Objects.requireNonNull(device, "device"),
                new RuntimeReader(
                        Objects.requireNonNull(yawDegrees, "yawDegrees"),
                        Objects.requireNonNull(angleDegrees, "angleDegrees")));
    }

    private static String sanitize(String key) {
        return key.toLowerCase(Locale.ROOT).replace(':', '_').replace('-', '_');
    }

    private HardwarePort.Can requireCan() {
        if (port instanceof HardwarePort.Can can) {
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

    private record RuntimeReader(DoubleSupplier yawDegrees, DoubleSupplier angleDegrees) {
    }
}
