package ca.frc6390.athena.hardware.runtime;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.IdentityHashMap;

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
    private final Map<Object, AutoCloseable> runtimeBindings = new IdentityHashMap<>();
    private final RuntimeScope runtimeScope = new RuntimeScope("hardware-" + Integer.toHexString(System.identityHashCode(this)));

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
        HardwareIdentity identity = HardwareIdentity.motor(device);
        MotorHandle existing = motors.get(identity);
        if (existing != null) {
            if (!existing.device().equals(device)) {
                throw new IllegalStateException("Conflicting motor declarations target " + identity.key()
                        + ". First declaration: " + existing.device()
                        + "; conflicting declaration: " + device);
            }
            return existing;
        }
        MotorHandle leader = device.follower() == null ? null : motor(device.follower().leader());
        MotorHandle handle = backends
                .motorBackendFor(device.kind())
                .orElseThrow(() -> new IllegalStateException(
                        backends.missingBackendMessage("motor", device.kind().key())))
                .create(device);
        handle.activate();
        if (leader != null) {
            handle.follow(leader, device.isInverted());
        }
        motors.put(identity, handle);
        runtimeBindings.computeIfAbsent(device, ignored -> device.bindRuntime(runtimeScope, handle));
        return handle;
    }

    @Override
    public synchronized EncoderHandle encoder(EncoderDevice device) {
        Objects.requireNonNull(device, "device");
        EncoderHandle handle;
        if (device.source() instanceof EncoderDevice.EncoderSource.IntegratedMotor integrated) {
            MotorHandle motor = motor(integrated.motor());
            handle = encoders.computeIfAbsent(HardwareIdentity.encoder(device), ignored -> {
                motor.enableIntegratedEncoder();
                return adjustable(new IntegratedEncoderHandle(device, motor));
            });
        } else if (device.source() instanceof EncoderDevice.EncoderSource.MotorAbsolute absolute) {
            MotorHandle motor = motor(absolute.motor());
            handle = encoders.computeIfAbsent(HardwareIdentity.encoder(device), ignored -> {
                motor.enableAbsoluteEncoder();
                return adjustable(new AbsoluteMotorEncoderHandle(device, motor));
            });
        } else {
            handle = encoders.computeIfAbsent(HardwareIdentity.encoder(device), ignored -> {
                EncoderHandle created = backends
                        .encoderBackendFor(device)
                        .orElseThrow(() -> new IllegalStateException(backends.missingBackendMessage(
                                "encoder", device.kind().key() + " over " + device.connection().identity())))
                        .create(device);
                created.activate();
                return adjustable(created);
            });
        }
        runtimeBindings.computeIfAbsent(device, ignored -> device.bindRuntime(runtimeScope, handle));
        return handle;
    }

    public RuntimeScope runtimeScope() {
        return runtimeScope;
    }

    private static EncoderHandle adjustable(EncoderHandle handle) {
        return handle.supportsPositionSetting() ? handle : new PositionAdjustableEncoderHandle(handle);
    }

    /**
     * Resolves an IMU declaration to its cached runtime handle.
     *
     * @param device IMU declaration
     * @return runtime IMU
     */
    public synchronized ImuHandle imu(ImuDevice device) {
        Objects.requireNonNull(device, "device");
        ImuHandle handle = imus.computeIfAbsent(HardwareIdentity.imu(device), ignored -> {
            ImuHandle created = backends
                    .imuBackendFor(device)
                    .orElseThrow(() -> new IllegalStateException(backends.missingBackendMessage(
                            "IMU", device.kind().key() + " over " + device.connection().identity())))
                    .create(device);
            created.activate();
            return created;
        });
        runtimeBindings.computeIfAbsent(device, ignored -> device.bindRuntime(runtimeScope, handle));
        return handle;
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
        closeAll(runtimeBindings);
        runtimeBindings.clear();
        closeAll(imus);
        closeAll(encoders);
        closeAll(motors);
    }

    private static void closeAll(Map<?, ?> handles) {
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

        @Override
        public void setPositionRotations(double rotations) {
            motor.setIntegratedPositionRotations(rotations);
        }

        @Override
        public boolean supportsPositionSetting() {
            return motor.supportsIntegratedPositionSetting();
        }
    }

    private static final class PositionAdjustableEncoderHandle implements EncoderHandle, AutoCloseable {
        private final EncoderHandle delegate;
        private volatile double softwareOffsetRotations;

        private PositionAdjustableEncoderHandle(EncoderHandle delegate) {
            this.delegate = Objects.requireNonNull(delegate, "delegate");
        }

        @Override
        public EncoderDevice device() {
            return delegate.device();
        }

        @Override
        public void refreshInputs() {
            delegate.refreshInputs();
        }

        @Override
        public double positionRotations() {
            return delegate.positionRotations() + softwareOffsetRotations;
        }

        @Override
        public double absolutePositionRotations() {
            return delegate.absolutePositionRotations();
        }

        @Override
        public double velocityRotationsPerSecond() {
            return delegate.velocityRotationsPerSecond();
        }

        @Override
        public synchronized void setPositionRotations(double rotations) {
            if (!Double.isFinite(rotations)) {
                throw new IllegalArgumentException("Encoder position must be finite.");
            }
            try {
                delegate.setPositionRotations(rotations);
                softwareOffsetRotations = 0.0;
            } catch (UnsupportedOperationException unsupported) {
                softwareOffsetRotations = rotations - delegate.positionRotations();
            }
        }

        @Override
        public boolean supportsPositionSetting() {
            return true;
        }

        @Override
        public void close() {
            if (delegate instanceof AutoCloseable closeable) {
                try {
                    closeable.close();
                } catch (InterruptedException exception) {
                    Thread.currentThread().interrupt();
                    throw new IllegalStateException("Interrupted while closing " + device().defaultName(), exception);
                } catch (Exception exception) {
                    throw new IllegalStateException("Failed to close " + device().defaultName(), exception);
                }
            }
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
