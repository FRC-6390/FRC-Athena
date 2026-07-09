package ca.frc6390.athena.hardware.runtime;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
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
    private final Map<HardwareIdentity, RuntimeException> refreshFailures = new LinkedHashMap<>();

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
    public synchronized MotorHandle motor(MotorDevice device) {
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
    public synchronized EncoderHandle encoder(EncoderDevice device) {
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
    public synchronized ImuHandle imu(ImuDevice device) {
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
        List<RefreshTask> tasks;
        synchronized (this) {
            tasks = new ArrayList<>(motors.size() + encoders.size() + imus.size());
            motors.forEach((identity, handle) -> tasks.add(new RefreshTask(identity, handle::refreshInputs)));
            encoders.forEach((identity, handle) -> tasks.add(new RefreshTask(identity, handle::refreshInputs)));
            imus.forEach((identity, handle) -> tasks.add(new RefreshTask(identity, handle::refreshInputs)));
        }
        Map<HardwareIdentity, RuntimeException> failures = new LinkedHashMap<>();
        tasks.forEach(task -> refresh(task.identity(), task.refresh(), failures));
        synchronized (this) {
            refreshFailures.clear();
            refreshFailures.putAll(failures);
        }
    }

    /**
     * Returns failures recorded during the latest input refresh.
     *
     * @return refresh failures
     */
    public synchronized List<RefreshFailure> refreshFailures() {
        return refreshFailures.entrySet().stream()
                .map(entry -> new RefreshFailure(entry.getKey(), entry.getValue()))
                .toList();
    }

    private static void refresh(
            HardwareIdentity identity,
            Runnable refresh,
            Map<HardwareIdentity, RuntimeException> failures) {
        try {
            refresh.run();
        } catch (RuntimeException exception) {
            failures.put(identity, exception);
        }
    }

    @Override
    public synchronized void close() {
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

    /**
     * A handle refresh failure captured without aborting the rest of the runtime refresh.
     *
     * @param identity hardware identity
     * @param exception thrown exception
     */
    public record RefreshFailure(HardwareIdentity identity, RuntimeException exception) {
        public RefreshFailure {
            Objects.requireNonNull(identity, "identity");
            Objects.requireNonNull(exception, "exception");
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

    private record RefreshTask(HardwareIdentity identity, Runnable refresh) {
        private RefreshTask {
            Objects.requireNonNull(identity, "identity");
            Objects.requireNonNull(refresh, "refresh");
        }
    }
}
