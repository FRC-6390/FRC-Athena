package ca.frc6390.athena.hardware.runtime;

import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;

import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.HardwareIdentity;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;

/**
 * Runtime-owned hardware graph that resolves declarations to cached handles.
 */
public final class HardwareGraph implements ActionContext, AutoCloseable {
    private final BackendRegistry backends;
    private final Map<HardwareIdentity, MotorHandle> motors = new LinkedHashMap<>();
    private final Map<HardwareIdentity, EncoderHandle> encoders = new LinkedHashMap<>();
    private final Map<HardwareIdentity, ImuHandle> imus = new LinkedHashMap<>();

    /**
     * Creates a graph using the global backend registry.
     *
     * @return hardware graph
     */
    public static HardwareGraph discovered() {
        return using(BackendRegistry.global());
    }

    /**
     * Creates a graph using an explicit backend registry.
     *
     * @param backends backend registry
     * @return hardware graph
     */
    public static HardwareGraph using(BackendRegistry backends) {
        return new HardwareGraph(backends);
    }

    /**
     * Creates a graph.
     *
     * @param backends backend registry
     */
    public HardwareGraph(BackendRegistry backends) {
        this.backends = Objects.requireNonNull(backends, "backends");
    }

    @Override
    public MotorHandle motor(MotorDevice device) {
        Objects.requireNonNull(device, "device");
        return motors.computeIfAbsent(HardwareIdentity.motor(device), ignored -> {
            MotorHandle handle = backends
                    .motorBackendFor(device.kind())
                    .orElseThrow(() -> new IllegalStateException("No motor backend for " + device.kind().key()))
                    .create(device);
            handle.activate();
            return handle;
        });
    }

    @Override
    public EncoderHandle encoder(EncoderDevice device) {
        Objects.requireNonNull(device, "device");
        if (device.source() instanceof EncoderDevice.EncoderSource.IntegratedMotor integrated) {
            return encoders.computeIfAbsent(
                    HardwareIdentity.encoder(device),
                    ignored -> new IntegratedEncoderHandle(device, motor(integrated.motor())));
        }
        if (device.source() instanceof EncoderDevice.EncoderSource.MotorAbsolute absolute) {
            return encoders.computeIfAbsent(
                    HardwareIdentity.encoder(device),
                    ignored -> new AbsoluteMotorEncoderHandle(device, motor(absolute.motor())));
        }
        return encoders.computeIfAbsent(HardwareIdentity.encoder(device), ignored -> {
            EncoderHandle handle = backends
                    .encoderBackendFor(device.kind())
                    .orElseThrow(() -> new IllegalStateException("No encoder backend for " + device.kind().key()))
                    .create(device);
            handle.activate();
            return handle;
        });
    }

    /**
     * Resolves an IMU declaration to its cached runtime handle.
     *
     * @param device IMU declaration
     * @return runtime IMU
     */
    public ImuHandle imu(ImuDevice device) {
        Objects.requireNonNull(device, "device");
        return imus.computeIfAbsent(HardwareIdentity.imu(device), ignored -> {
            ImuHandle handle = backends
                    .imuBackendFor(device.kind())
                    .orElseThrow(() -> new IllegalStateException("No IMU backend for " + device.kind().key()))
                    .create(device);
            handle.activate();
            return handle;
        });
    }

    /**
     * Refreshes all cached hardware input snapshots once for the current runtime cycle.
     */
    public void refreshInputs() {
        motors.values().forEach(MotorHandle::refreshInputs);
        encoders.values().forEach(EncoderHandle::refreshInputs);
        imus.values().forEach(ImuHandle::refreshInputs);
    }

    @Override
    public void close() {
        closeAll(imus);
        closeAll(encoders);
        closeAll(motors);
    }

    private static void closeAll(Map<HardwareIdentity, ?> handles) {
        for (Object handle : handles.values()) {
            if (handle instanceof AutoCloseable closeable) {
                try {
                    closeable.close();
                } catch (Exception exception) {
                    throw new IllegalStateException("Failed to close hardware handle.", exception);
                }
            }
        }
    }

    private record IntegratedEncoderHandle(EncoderDevice device, MotorHandle motor) implements EncoderHandle {
        @Override
        public double positionRotations() {
            return motor.integratedPositionRotations();
        }

        @Override
        public double velocityRotationsPerSecond() {
            return motor.integratedVelocityRotationsPerSecond();
        }
    }

    private record AbsoluteMotorEncoderHandle(EncoderDevice device, MotorHandle motor) implements EncoderHandle {
        @Override
        public double positionRotations() {
            return motor.absolutePositionRotations();
        }

        @Override
        public double absolutePositionRotations() {
            return motor.absolutePositionRotations();
        }

        @Override
        public double velocityRotationsPerSecond() {
            return motor.absoluteVelocityRotationsPerSecond();
        }
    }
}
