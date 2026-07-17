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
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.api.FailurePolicy;

/**
 * Runtime-owned hardware graph that resolves declarations to cached handles.
 */
public final class HardwareGraph implements ActionContext, AutoCloseable {
    private final BackendRegistry backends;
    private final Map<HardwareIdentity, MotorHandle> motors = new LinkedHashMap<>();
    private final Map<HardwareIdentity, List<ActionContext.SoftwareMotorFollower>> softwareFollowers =
            new LinkedHashMap<>();
    private final Map<HardwareIdentity, EncoderHandle> encoders = new LinkedHashMap<>();
    private final Map<HardwareIdentity, ImuHandle> imus = new LinkedHashMap<>();
    private final Map<HardwareIdentity, RuntimeException> refreshFailures = new LinkedHashMap<>();
    private final Map<Object, RuntimeException> bindingFailures = new LinkedHashMap<>();
    private final Map<Object, RuntimeException> operationFailures = new LinkedHashMap<>();
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
        if (existing != null && !existing.device().equals(device)) {
            throw new IllegalStateException("Conflicting motor declarations target " + identity.key()
                    + ". First declaration: " + existing.device()
                    + "; conflicting declaration: " + device);
        }
        try {
            return createMotor(device);
        } catch (RuntimeException exception) {
            if (device.failurePolicy() == FailurePolicy.PANIC) {
                throw exception;
            }
            bindingFailures.put(device, exception);
            MotorHandle unavailable = new UnavailableMotorHandle(device);
            motors.putIfAbsent(HardwareIdentity.motor(device), unavailable);
            runtimeBindings.computeIfAbsent(device, ignored -> device.bindRuntime(runtimeScope, unavailable));
            return unavailable;
        }
    }

    private MotorHandle createMotor(MotorDevice device) {
        HardwareIdentity identity = HardwareIdentity.motor(device);
        MotorHandle existing = motors.get(identity);
        if (existing != null) {
            return existing;
        }
        MotorHandle leader = device.follower() == null ? null : motor(device.follower().leader());
        MotorBackend backend = backends
                .motorBackendFor(device.kind())
                .orElseThrow(() -> new IllegalStateException(
                        backends.missingBackendMessage("motor", device.kind().key())));
        boolean softwareFollow = leader != null
                && !backend.supportsHardwareFollowing(device, device.follower().leader());
        MotorDevice backendDevice = softwareFollow
                ? device.independent().inverted(false)
                : device;
        MotorHandle created = backend.create(backendDevice);
        MotorHandle handle = softwareFollow
                ? new SoftwareFollowerMotorHandle(device, created)
                : created;
        handle.activate();
        if (leader != null && !softwareFollow) {
            handle.follow(backendHandle(leader), device.isInverted());
        } else if (softwareFollow) {
            softwareFollowers.computeIfAbsent(
                    HardwareIdentity.motor(device.follower().leader()),
                    ignored -> new ArrayList<>())
                    .add(new ActionContext.SoftwareMotorFollower(device, handle));
        }
        motors.put(identity, handle);
        runtimeBindings.computeIfAbsent(device, ignored -> device.bindRuntime(runtimeScope, handle));
        return handle;
    }

    @Override
    public synchronized List<ActionContext.SoftwareMotorFollower> softwareFollowers(MotorDevice leader) {
        return List.copyOf(softwareFollowers.getOrDefault(HardwareIdentity.motor(leader), List.of()));
    }

    @Override
    public synchronized EncoderHandle encoder(EncoderDevice device) {
        Objects.requireNonNull(device, "device");
        try {
            return createEncoder(device);
        } catch (RuntimeException exception) {
            if (device.failurePolicy() == FailurePolicy.PANIC) {
                throw exception;
            }
            bindingFailures.put(device, exception);
            EncoderHandle unavailable = new UnavailableEncoderHandle(device);
            encoders.putIfAbsent(HardwareIdentity.encoder(device), unavailable);
            runtimeBindings.computeIfAbsent(device, ignored -> device.bindRuntime(runtimeScope, unavailable));
            return unavailable;
        }
    }

    private EncoderHandle createEncoder(EncoderDevice device) {
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

    private static MotorHandle backendHandle(MotorHandle handle) {
        return handle instanceof SoftwareFollowerMotorHandle software
                ? software.backendHandle()
                : handle;
    }

    /**
     * Resolves an IMU declaration to its cached runtime handle.
     *
     * @param device IMU declaration
     * @return runtime IMU
     */
    public synchronized ImuHandle imu(ImuDevice device) {
        Objects.requireNonNull(device, "device");
        try {
            return createImu(device);
        } catch (RuntimeException exception) {
            if (device.failurePolicy() == FailurePolicy.PANIC) {
                throw exception;
            }
            bindingFailures.put(device, exception);
            ImuHandle unavailable = new UnavailableImuHandle(device);
            imus.putIfAbsent(HardwareIdentity.imu(device), unavailable);
            runtimeBindings.computeIfAbsent(device, ignored -> device.bindRuntime(runtimeScope, unavailable));
            return unavailable;
        }
    }

    private ImuHandle createImu(ImuDevice device) {
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
            motors.forEach((identity, handle) -> tasks.add(new RefreshTask(identity, handle.device(), handle::refreshInputs)));
            encoders.forEach((identity, handle) -> tasks.add(new RefreshTask(identity, handle.device(), handle::refreshInputs)));
            imus.forEach((identity, handle) -> tasks.add(new RefreshTask(identity, handle.device(), handle::refreshInputs)));
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
                .map(entry -> new RefreshFailure(
                        entry.getKey(), declaration(entry.getKey()), entry.getValue()))
                .toList();
    }

    /** Returns declarations that could not be bound during graph construction. */
    public synchronized List<BindingFailure> bindingFailures() {
        return bindingFailures.entrySet().stream()
                .map(entry -> new BindingFailure(entry.getKey(), entry.getValue()))
                .toList();
    }

    @Override
    public synchronized void hardwareFailure(Object declaration, RuntimeException exception) {
        operationFailures.put(
                Objects.requireNonNull(declaration, "declaration"),
                Objects.requireNonNull(exception, "exception"));
    }

    /** Returns and clears output/setup failures reported since the previous drain. */
    public synchronized List<BindingFailure> drainOperationFailures() {
        List<BindingFailure> failures = operationFailures.entrySet().stream()
                .map(entry -> new BindingFailure(entry.getKey(), entry.getValue()))
                .toList();
        operationFailures.clear();
        return failures;
    }

    private Object declaration(HardwareIdentity identity) {
        if (motors.containsKey(identity)) return motors.get(identity).device();
        if (encoders.containsKey(identity)) return encoders.get(identity).device();
        if (imus.containsKey(identity)) return imus.get(identity).device();
        throw new IllegalStateException("No declaration exists for " + identity.key());
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
        softwareFollowers.clear();
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
    public record RefreshFailure(HardwareIdentity identity, Object declaration, RuntimeException exception) {
        public RefreshFailure {
            Objects.requireNonNull(identity, "identity");
            Objects.requireNonNull(declaration, "declaration");
            Objects.requireNonNull(exception, "exception");
        }
    }

    /** A declaration that could not create its backend handle. */
    public record BindingFailure(Object declaration, RuntimeException exception) {
        public BindingFailure {
            Objects.requireNonNull(declaration, "declaration");
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

    private record RefreshTask(HardwareIdentity identity, Object declaration, Runnable refresh) {
        private RefreshTask {
            Objects.requireNonNull(identity, "identity");
            Objects.requireNonNull(declaration, "declaration");
            Objects.requireNonNull(refresh, "refresh");
        }
    }

    private record UnavailableMotorHandle(MotorDevice device) implements MotorHandle {
        @Override public void setPercentOutput(double percent) { }
        @Override public void setVoltage(double volts) { }
        @Override public double appliedVoltage() { return 0.0; }
        @Override public double supplyCurrentAmps() { return 0.0; }
        @Override public double statorCurrentAmps() { return 0.0; }
        @Override public double integratedPositionRotations() { return 0.0; }
        @Override public double integratedVelocityRotationsPerSecond() { return 0.0; }
        @Override public double absolutePositionRotations() { return 0.0; }
        @Override public double absoluteVelocityRotationsPerSecond() { return 0.0; }
        @Override public void setIntegratedPositionRotations(double rotations) { }
        @Override public boolean supportsIntegratedPositionSetting() { return true; }
    }

    private record UnavailableEncoderHandle(EncoderDevice device) implements EncoderHandle {
        @Override public double positionRotations() { return 0.0; }
        @Override public double absolutePositionRotations() { return 0.0; }
        @Override public double velocityRotationsPerSecond() { return 0.0; }
        @Override public void setPositionRotations(double rotations) { }
        @Override public boolean supportsPositionSetting() { return true; }
    }

    private record UnavailableImuHandle(ImuDevice device) implements ImuHandle {
        @Override public double yawDegrees() { return 0.0; }
        @Override public double pitchDegrees() { return 0.0; }
        @Override public double rollDegrees() { return 0.0; }
        @Override public double yawRateDegreesPerSecond() { return 0.0; }
        @Override public double linearAccelerationXG() { return 0.0; }
        @Override public double linearAccelerationYG() { return 0.0; }
        @Override public double linearAccelerationZG() { return 0.0; }
        @Override public void setYawDegrees(double yawDegrees) { }
    }
}
