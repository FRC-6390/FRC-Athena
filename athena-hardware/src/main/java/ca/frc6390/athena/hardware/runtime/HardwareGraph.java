package ca.frc6390.athena.hardware.runtime;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.IdentityHashMap;
import java.util.HashMap;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import java.util.function.LongSupplier;

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
 * Runtime-owned hardware graph that resolves declarations to cached handles. Explicit standalone
 * graphs prepare initial declarations synchronously, outside the graph monitor, for setup/tooling
 * compatibility. Call {@link #asynchronousBindings()} before registration to keep vendor construction
 * off the caller thread. RobotRuntime selects this for real hardware and all late runtime bindings.
 */
public final class HardwareGraph implements ActionContext, AutoCloseable {
    private final BackendRegistry backends;
    private final Map<HardwareIdentity, MotorHandle> motors = new LinkedHashMap<>();
    private final Map<HardwareIdentity, List<ActionContext.SoftwareMotorFollower>> softwareFollowers =
            new LinkedHashMap<>();
    private final Map<HardwareIdentity, List<ActionContext.SoftwareMotorFollower>> softwareFollowerViews =
            new LinkedHashMap<>();
    private final Map<HardwareIdentity, FollowerView> followerConstraintViews = new LinkedHashMap<>();
    private long followerRevision;
    private final Map<HardwareIdentity, EncoderHandle> encoders = new LinkedHashMap<>();
    private final Map<HardwareIdentity, ImuHandle> imus = new LinkedHashMap<>();
    private final Map<HardwareIdentity, MotorGroup> motorGroups = new LinkedHashMap<>();
    private volatile MotorGroup[] motorGroupArray = new MotorGroup[0];
    private final MotorGroup.Wiring groupWiring = new GroupWiring();
    private final Map<MotorDevice, HardwareIdentity> motorIdentities = new IdentityHashMap<>();
    private final Map<EncoderDevice, HardwareIdentity> encoderIdentities = new IdentityHashMap<>();
    private final Map<ImuDevice, HardwareIdentity> imuIdentities = new IdentityHashMap<>();
    private final Map<HardwareIdentity, RuntimeException> refreshFailures = new LinkedHashMap<>();
    private List<RefreshFailure> refreshFailureSnapshot = List.of();
    private boolean refreshFailureSnapshotDirty;
    private final Map<HardwareIdentity, RefreshBackoff> refreshBackoffs = new LinkedHashMap<>();
    private RefreshTask[] refreshTasks = new RefreshTask[0];
    private int refreshTaskHandleCount = -1;
    private SnapshotSources snapshotSources = SnapshotSources.empty();
    private SnapshotSources snapshotEntrySources;
    private SnapshotEntry[] motorSnapshotEntries;
    private SnapshotEntry[] encoderSnapshotEntries;
    private SnapshotEntry[] imuSnapshotEntries;
    private int activeRefreshCycles;
    private boolean refreshCycleInFlight;
    private boolean closed;
    private boolean asynchronousBindings;
    private long refreshSequence;
    private volatile HardwareCycleSnapshot cycleSnapshot = HardwareCycleSnapshot.empty();
    private final Map<Object, RuntimeException> bindingFailures = new IdentityHashMap<>();
    private final Map<Object, RefreshBackoff> bindingRetryBackoffs = new IdentityHashMap<>();
    private final Map<Object, RuntimeException> operationFailures = new LinkedHashMap<>();
    private final Map<Object, AutoCloseable> runtimeBindings = new IdentityHashMap<>();
    private final Map<Object, Object> runtimeBindingHandles = new IdentityHashMap<>();
    /**
     * Motor handles resolved by {@link #motor(MotorDevice)}, keyed by declaration. An entry is valid only while
     * {@link #motorEpoch} equals its epoch: the epoch moves whenever the motor map or a runtime binding changes
     * (creation, failover adoption, rebinding, close), so a stale handle is never returned.
     */
    private final Map<MotorDevice, MotorLookup> motorLookups = new IdentityHashMap<>();
    private long motorEpoch;
    /** Placeholder handle for each declaration that failed to bind; the loop uses it until the binder succeeds. */
    private final Map<Object, Object> placeholders = new IdentityHashMap<>();
    /** Declarations whose background attempt is queued or running, with the identity each one binds. */
    private final Map<Object, HardwareIdentity> pendingBindings = new IdentityHashMap<>();
    private final java.util.Set<Object> runningBindings = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    /** How many pending declarations bind each identity. */
    private final Map<HardwareIdentity, Integer> pendingIdentities = new HashMap<>();
    private ExecutorService binderExecutor;
    private LongSupplier clock = System::nanoTime;

    private record MotorLookup(MotorHandle handle, long epoch) {
    }
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

    /** The graph monitor publishes all motor, encoder and IMU binding replacements together. */
    @Override
    public synchronized long bindingRevision() {
        return motorEpoch;
    }

    @Override
    public MotorHandle motor(MotorDevice device) {
        Objects.requireNonNull(device, "device");
        HardwareIdentity identity;
        synchronized (this) {
        if (closed) throw new IllegalStateException("Hardware graph is closed.");
        // Steady state: the same declaration resolves to the same handle every cycle. The cached entry is only
        // used while no binding failure is recorded for the declaration.
        MotorLookup cached = motorLookups.get(device);
        if (cached != null && cached.epoch() == motorEpoch && !bindingFailures.containsKey(device)) {
            return cached.handle();
        }
        identity = identity(device);
        MotorHandle existing = motors.get(identity);
        if (existing != null && existing.device() != device && !existing.device().equals(device)) {
            throw new IllegalStateException("Conflicting motor declarations target " + identity.key()
                    + ". First declaration: " + existing.device()
                    + "; conflicting declaration: " + device);
        }
        if (existing != null) {
            // Already bound by another declaration with the same identity: bind to it without any vendor call.
            MotorHandle bound = createMotor(device);
            motorLookups.put(device, new MotorLookup(bound, motorEpoch));
            return bound;
        }
        if (bindingFailures.containsKey(device) || isIdentityPending(identity)) {
            // A failed or background-pending declaration is never re-created inline: that would put a vendor create
            // on the caller's thread every cycle. The binder retries it; until then the unavailable handle is returned.
            return unavailableMotor(device);
        }
        if (asynchronousBindings) {
            if (device.follower() != null) motor(device.follower().leader());
            submitBinding(device);
            return unavailableMotor(device);
        }
        pendingIdentities.merge(identity, 1, Integer::sum);
        }
        PreparedMotor prepared = null;
        try {
            MotorHandle leader = device.follower() == null ? null : motor(device.follower().leader());
            prepared = prepareMotor(device, leader);
            synchronized (this) {
                if (closed) throw new IllegalStateException("Hardware graph is closed.");
                MotorHandle handle = installMotor(device, prepared);
                motorLookups.put(device, new MotorLookup(handle, motorEpoch));
                prepared = null;
                return handle;
            }
        } catch (RuntimeException exception) {
            synchronized (this) {
            if (device.failurePolicy() == FailurePolicy.PANIC) {
                throw exception;
            }
            recordBindingFailure(device, exception);
            return unavailableMotor(device);
            }
        } finally {
            if (prepared != null) closeHandle(prepared.created());
            synchronized (this) {
                pendingIdentities.computeIfPresent(identity, (key, count) -> count > 1 ? count - 1 : null);
                notifyAll();
            }
        }
    }

    /** Uses background preparation for new declarations. Existing cached handles remain immediate. */
    public synchronized HardwareGraph asynchronousBindings() {
        asynchronousBindings = true;
        return this;
    }

    private MotorHandle unavailableMotor(MotorDevice device) {
        MotorHandle placeholder = (MotorHandle) placeholders.computeIfAbsent(
                device, ignored -> new UnavailableMotorHandle(device));
        bindRuntime(device, placeholder);
        return placeholder;
    }

    /** Rebinds an already installed handle; this bookkeeping never performs vendor work. */
    private MotorHandle createMotor(MotorDevice device) {
        HardwareIdentity identity = identity(device);
        MotorHandle existing = motors.get(identity);
        if (existing != null) {
            bindRuntime(device, existing);
            clearFailure(device);
            return existing;
        }
        throw new IllegalStateException("Motor is not installed: " + identity.key());
    }

    /** Vendor work for a motor, with no graph state read or written. The leader handle is passed in. */
    private PreparedMotor prepareMotor(MotorDevice device, MotorHandle leader) {
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
        }
        return new PreparedMotor(created, handle, softwareFollow);
    }

    /** Publishes a prepared motor into the graph. Caller holds the graph lock; makes no vendor calls. */
    private MotorHandle installMotor(MotorDevice device, PreparedMotor prepared) {
        HardwareIdentity identity = identity(device);
        MotorHandle handle = prepared.handle();
        boolean declaresLeader = device.follower() != null;
        ActionContext.SoftwareMotorFollower softwareFollower = null;
        if (declaresLeader && prepared.softwareFollow()) {
            HardwareIdentity leaderIdentity = identity(device.follower().leader());
            softwareFollower = new ActionContext.SoftwareMotorFollower(device, handle);
            softwareFollowers.computeIfAbsent(leaderIdentity, ignored -> new ArrayList<>())
                    .add(softwareFollower);
            softwareFollowerViews.remove(leaderIdentity);
        }
        MotorGroup ledGroup = motorGroups.get(identity);
        if (ledGroup != null) {
            handle = ledGroup.adoptLeader(handle);
        } else if (declaresLeader) {
            MotorGroup followedGroup = motorGroups.get(identity(device.follower().leader()));
            if (followedGroup != null) followedGroup.adoptFollower(device, identity, handle, softwareFollower);
        }
        motors.put(identity, handle);
        motorEpoch++;
        bindRuntime(device, handle);
        clearFailure(device);
        return handle;
    }

    @Override
    public synchronized List<ActionContext.SoftwareMotorFollower> softwareFollowers(MotorDevice leader) {
        HardwareIdentity leaderIdentity = identity(leader);
        List<ActionContext.SoftwareMotorFollower> followers = softwareFollowers.get(leaderIdentity);
        if (followers == null || followers.isEmpty()) return List.of();
        List<ActionContext.SoftwareMotorFollower> view = softwareFollowerViews.get(leaderIdentity);
        if (view != null) return view;
        return softwareFollowerViews.computeIfAbsent(leaderIdentity, ignored -> List.copyOf(followers));
    }

    @Override
    public synchronized List<ActionContext.MotorFollower> followerConstraints(MotorDevice leader) {
        HardwareIdentity leaderIdentity = identity(leader);
        FollowerView cached = followerConstraintViews.get(leaderIdentity);
        if (cached != null && cached.motorEpoch == motorEpoch && cached.revision == followerRevision) {
            return cached.followers;
        }
        List<ActionContext.MotorFollower> constraints = new ArrayList<>();
        java.util.Set<HardwareIdentity> visited = new java.util.HashSet<>();
        visited.add(leaderIdentity);
        MotorHandle leaderHandle = motors.get(leaderIdentity);
        if (leaderHandle != null) collectFollowerConstraints(leaderIdentity, leaderHandle, constraints, visited, true);
        List<ActionContext.MotorFollower> view = List.copyOf(constraints);
        followerConstraintViews.put(leaderIdentity, new FollowerView(motorEpoch, followerRevision, view));
        return view;
    }

    private void collectFollowerConstraints(HardwareIdentity parent, MotorHandle parentFrame,
            List<ActionContext.MotorFollower> constraints, java.util.Set<HardwareIdentity> visited, boolean root) {
        MotorGroup group = motorGroups.get(parent);
        if (group != null) {
            for (ActionContext.MotorFollower follower : group.followerConstraints()) {
                HardwareIdentity followerIdentity = identity(follower.device());
                // The recovered declared leader is also a physical follower; retain its separate backend view.
                if (followerIdentity.equals(parent) || visited.add(followerIdentity)) constraints.add(follower);
            }
            // Group members follow the active leader regardless of their original declaration. Descendants
            // outside the group still follow their declared parent and inherit that parent's voltage frame.
            for (ActionContext.MotorFollower follower : group.followerConstraints()) {
                HardwareIdentity followerIdentity = identity(follower.device());
                if (!followerIdentity.equals(parent)) {
                    collectDeclaredFollowers(followerIdentity, follower.handle(), constraints, visited, false);
                }
            }
            return;
        }
        collectDeclaredFollowers(parent, parentFrame, constraints, visited, root);
    }

    private void collectDeclaredFollowers(HardwareIdentity parent, MotorHandle parentFrame,
            List<ActionContext.MotorFollower> constraints, java.util.Set<HardwareIdentity> visited, boolean root) {
        for (Map.Entry<HardwareIdentity, MotorHandle> entry : motors.entrySet()) {
            MotorDevice device = entry.getValue().device();
            if (device.follower() == null || !identity(device.follower().leader()).equals(parent)
                    || !visited.add(entry.getKey())) continue;
            ActionContext.MotorFollower follower = null;
            for (ActionContext.SoftwareMotorFollower software : softwareFollowers.getOrDefault(parent, List.of())) {
                if (identity(software.device()).equals(entry.getKey())) {
                    follower = root ? software : new ActionContext.SoftwareMotorFollower(device,
                            new ChainedFollowerMotorHandle(software.handle(), parentFrame));
                    break;
                }
            }
            if (follower == null) {
                MotorHandle backend = entry.getValue();
                follower = FollowerConstraint.of(device, backend,
                        () -> parentFrame.voltageCommandScale() * backend.hardwareFollowerVoltageScale());
            }
            constraints.add(follower);
            collectFollowerConstraints(entry.getKey(), follower.handle(), constraints, visited, false);
        }
    }

    private record FollowerView(long motorEpoch, long revision, List<ActionContext.MotorFollower> followers) { }

    @Override
    public EncoderHandle encoder(EncoderDevice device) {
        Objects.requireNonNull(device, "device");
        HardwareIdentity identity;
        synchronized (this) {
        if (closed) throw new IllegalStateException("Hardware graph is closed.");
        identity = identity(device);
        EncoderHandle existing = encoders.get(identity);
        if (existing != null) {
            bindRuntime(device, existing);
            clearFailure(device);
            return existing;
        }
        if (bindingFailures.containsKey(device) || isIdentityPending(identity)) {
            return unavailableEncoder(device);
        }
        if (asynchronousBindings) {
            MotorDevice source = sourceMotorDevice(device);
            if (source != null) motor(source);
            submitBinding(device);
            return unavailableEncoder(device);
        }
        pendingIdentities.merge(identity, 1, Integer::sum);
        }
        EncoderHandle prepared = null;
        try {
            prepared = prepareEncoder(device, sourceMotor(device));
            synchronized (this) {
                if (closed) throw new IllegalStateException("Hardware graph is closed.");
                EncoderHandle installed = installEncoder(device, prepared);
                prepared = null;
                return installed;
            }
        } catch (RuntimeException exception) {
            synchronized (this) {
            if (device.failurePolicy() == FailurePolicy.PANIC) {
                throw exception;
            }
            recordBindingFailure(device, exception);
            return unavailableEncoder(device);
            }
        } finally {
            if (prepared != null) closeHandle(prepared);
            synchronized (this) {
                pendingIdentities.computeIfPresent(identity, (key, count) -> count > 1 ? count - 1 : null);
                notifyAll();
            }
        }
    }

    private EncoderHandle unavailableEncoder(EncoderDevice device) {
        EncoderHandle placeholder = (EncoderHandle) placeholders.computeIfAbsent(
                device, ignored -> new UnavailableEncoderHandle(device));
        bindRuntime(device, placeholder);
        return placeholder;
    }

    /** The motor an encoder reads from, or null for a standalone encoder. Bound synchronously. */
    private MotorHandle sourceMotor(EncoderDevice device) {
        MotorDevice source = sourceMotorDevice(device);
        return source == null ? null : motor(source);
    }

    /** The motor an encoder reads from, found without creating it: the binder waits until it is bound. */
    private MotorHandle boundSourceMotor(EncoderDevice device) {
        MotorDevice source = sourceMotorDevice(device);
        if (source == null) {
            return null;
        }
        MotorHandle handle = motors.get(identity(source));
        if (handle == null) {
            throw new DependencyPending("Motor " + HardwareIdentity.motor(source).key() + " is not bound yet.");
        }
        return handle;
    }

    private static MotorDevice sourceMotorDevice(EncoderDevice device) {
        if (device.source() instanceof EncoderDevice.EncoderSource.IntegratedMotor integrated) {
            return integrated.motor();
        }
        if (device.source() instanceof EncoderDevice.EncoderSource.MotorAbsolute absolute) {
            return absolute.motor();
        }
        return null;
    }

    /** Vendor work for an encoder. Integrated and absolute encoders are configured on their motor. */
    private EncoderHandle prepareEncoder(EncoderDevice device, MotorHandle motor) {
        if (device.source() instanceof EncoderDevice.EncoderSource.IntegratedMotor) {
            motor.enableIntegratedEncoder();
            return adjustable(new IntegratedEncoderHandle(device, motor));
        }
        if (device.source() instanceof EncoderDevice.EncoderSource.MotorAbsolute) {
            motor.enableAbsoluteEncoder();
            return adjustable(new AbsoluteMotorEncoderHandle(device, motor));
        }
        EncoderHandle created = backends
                .encoderBackendFor(device)
                .orElseThrow(() -> new IllegalStateException(backends.missingBackendMessage(
                        "encoder", device.kind().key() + " over " + device.connection().identity())))
                .create(device);
        created.activate();
        return adjustable(created);
    }

    /** Publishes a prepared encoder. Caller holds the graph lock. */
    private EncoderHandle installEncoder(EncoderDevice device, EncoderHandle prepared) {
        HardwareIdentity identity = identity(device);
        EncoderHandle installed = encoders.get(identity);
        if (installed == null) {
            installed = prepared;
            encoders.put(identity, installed);
        } else if (installed != prepared) {
            throw new IllegalStateException("Encoder was concurrently bound: " + identity.key());
        }
        bindRuntime(device, installed);
        clearFailure(device);
        return installed;
    }

    public RuntimeScope runtimeScope() {
        return runtimeScope;
    }

    private static EncoderHandle adjustable(EncoderHandle handle) {
        return handle.supportsPositionSetting() ? handle : new PositionAdjustableEncoderHandle(handle);
    }

    private static MotorHandle backendHandle(MotorHandle handle) {
        if (handle instanceof FailoverMotorHandle failover) handle = failover.original();
        return handle instanceof SoftwareFollowerMotorHandle software
                ? software.backendHandle()
                : handle;
    }

    /**
     * Declares how a leader-and-followers motor group responds when one of its motors dies, and returns its
     * runtime health. The group is the policy's leader and every motor declared with {@code .follow(leader)}.
     * Declare it before the motors are resolved when possible; a group declared later takes over the existing
     * handles. Declaring the same leader twice returns the first group.
     *
     * @param policy group policy
     * @return group runtime
     */
    public synchronized MotorGroup motorGroup(MotorGroupPolicy policy) {
        Objects.requireNonNull(policy, "policy");
        HardwareIdentity leaderIdentity = identity(policy.leader());
        MotorGroup existing = motorGroups.get(leaderIdentity);
        if (existing != null) return existing;
        MotorGroup group = new MotorGroup(policy, leaderIdentity, groupWiring);
        motorGroups.put(leaderIdentity, group);
        motorGroupArray = motorGroups.values().toArray(MotorGroup[]::new);
        MotorHandle leaderHandle = motors.get(leaderIdentity);
        if (leaderHandle != null) adoptExisting(group, leaderHandle);
        return group;
    }

    /** Returns whether a motor group handles this failure, so the motor's failure policy must not apply. */
    public synchronized boolean motorFailureHandled(Object declaration) {
        if (!(declaration instanceof MotorDevice motor) || motorGroupArray.length == 0) return false;
        HardwareIdentity identity = identity(motor);
        for (MotorGroup group : motorGroupArray) {
            if (group.absorbs(identity)) return true;
        }
        return false;
    }

    private void adoptExisting(MotorGroup group, MotorHandle leaderHandle) {
        HardwareIdentity leaderIdentity = group.leaderIdentity();
        FailoverMotorHandle failover = group.adoptLeader(leaderHandle);
        motors.put(leaderIdentity, failover);
        motorEpoch++;
        for (Map.Entry<Object, Object> entry : List.copyOf(runtimeBindingHandles.entrySet())) {
            if (entry.getValue() == leaderHandle && entry.getKey() instanceof MotorDevice device) {
                bindRuntime(device, failover);
            }
        }
        for (Map.Entry<HardwareIdentity, EncoderHandle> entry : encoders.entrySet()) {
            EncoderDevice encoder = entry.getValue().device();
            if (!(encoder.source() instanceof EncoderDevice.EncoderSource.IntegratedMotor integrated)
                    || !identity(integrated.motor()).equals(leaderIdentity)) {
                continue;
            }
            EncoderHandle previous = entry.getValue();
            EncoderHandle rebuilt = adjustable(new IntegratedEncoderHandle(encoder, failover));
            entry.setValue(rebuilt);
            for (Map.Entry<Object, Object> binding : List.copyOf(runtimeBindingHandles.entrySet())) {
                if (binding.getValue() == previous && binding.getKey() instanceof EncoderDevice device) {
                    bindRuntime(device, rebuilt);
                }
            }
        }
        List<ActionContext.SoftwareMotorFollower> mirrored = softwareFollowers.getOrDefault(leaderIdentity, List.of());
        for (Map.Entry<HardwareIdentity, MotorHandle> entry : motors.entrySet()) {
            MotorDevice device = entry.getValue().device();
            if (device.follower() == null || !identity(device.follower().leader()).equals(leaderIdentity)) continue;
            ActionContext.SoftwareMotorFollower software = null;
            for (ActionContext.SoftwareMotorFollower follower : mirrored) {
                if (follower.handle() == entry.getValue()) software = follower;
            }
            group.adoptFollower(device, entry.getKey(), entry.getValue(), software);
        }
        refreshTaskHandleCount = -1;
    }

    /**
     * Resolves an IMU declaration to its cached runtime handle.
     *
     * @param device IMU declaration
     * @return runtime IMU
     */
    @Override
    public ImuHandle imu(ImuDevice device) {
        Objects.requireNonNull(device, "device");
        HardwareIdentity identity;
        synchronized (this) {
        if (closed) throw new IllegalStateException("Hardware graph is closed.");
        identity = identity(device);
        ImuHandle existing = imus.get(identity);
        if (existing != null) {
            bindRuntime(device, existing);
            clearFailure(device);
            return existing;
        }
        if (bindingFailures.containsKey(device) || isIdentityPending(identity)) {
            return unavailableImu(device);
        }
        if (asynchronousBindings) {
            submitBinding(device);
            return unavailableImu(device);
        }
        pendingIdentities.merge(identity, 1, Integer::sum);
        }
        ImuHandle prepared = null;
        try {
            prepared = prepareImu(device);
            synchronized (this) {
                if (closed) throw new IllegalStateException("Hardware graph is closed.");
                ImuHandle installed = installImu(device, prepared);
                prepared = null;
                return installed;
            }
        } catch (RuntimeException exception) {
            synchronized (this) {
            if (device.failurePolicy() == FailurePolicy.PANIC) {
                throw exception;
            }
            recordBindingFailure(device, exception);
            return unavailableImu(device);
            }
        } finally {
            if (prepared != null) closeHandle(prepared);
            synchronized (this) {
                pendingIdentities.computeIfPresent(identity, (key, count) -> count > 1 ? count - 1 : null);
                notifyAll();
            }
        }
    }

    private ImuHandle unavailableImu(ImuDevice device) {
        ImuHandle placeholder = (ImuHandle) placeholders.computeIfAbsent(
                device, ignored -> new UnavailableImuHandle(device));
        bindRuntime(device, placeholder);
        return placeholder;
    }

    private ImuHandle prepareImu(ImuDevice device) {
        ImuHandle created = backends
                .imuBackendFor(device)
                .orElseThrow(() -> new IllegalStateException(backends.missingBackendMessage(
                        "IMU", device.kind().key() + " over " + device.connection().identity())))
                .create(device);
        created.activate();
        return created;
    }

    private ImuHandle installImu(ImuDevice device, ImuHandle prepared) {
        HardwareIdentity identity = identity(device);
        ImuHandle installed = imus.get(identity);
        if (installed == null) {
            installed = prepared;
            imus.put(identity, installed);
        } else if (installed != prepared) {
            throw new IllegalStateException("IMU was concurrently bound: " + identity.key());
        }
        bindRuntime(device, installed);
        clearFailure(device);
        return installed;
    }

    private void bindRuntime(MotorDevice device, MotorHandle handle) {
        // Checked before the binding lambda is created so per-cycle lookups of a bound device do not allocate.
        if (runtimeBindingHandles.get(device) == handle) return;
        replaceRuntimeBinding(device, handle, () -> device.bindRuntime(runtimeScope, handle));
    }

    private void bindRuntime(EncoderDevice device, EncoderHandle handle) {
        if (runtimeBindingHandles.get(device) == handle) return;
        replaceRuntimeBinding(device, handle, () -> device.bindRuntime(runtimeScope, handle));
    }

    private void bindRuntime(ImuDevice device, ImuHandle handle) {
        if (runtimeBindingHandles.get(device) == handle) return;
        replaceRuntimeBinding(device, handle, () -> device.bindRuntime(runtimeScope, handle));
    }

    private void replaceRuntimeBinding(Object device, Object handle, RuntimeBindingFactory factory) {
        if (runtimeBindingHandles.get(device) == handle) return;
        motorEpoch++;
        AutoCloseable previous = runtimeBindings.remove(device);
        if (previous != null) closeBinding(previous);
        runtimeBindings.put(device, factory.bind());
        runtimeBindingHandles.put(device, handle);
    }

    private static void closeBinding(AutoCloseable binding) {
        try {
            binding.close();
        } catch (Exception exception) {
            throw new IllegalStateException("Failed to replace hardware runtime binding.", exception);
        }
    }

    /**
     * Refreshes all cached hardware input snapshots once for the current runtime cycle.
     */
    public void refreshInputs() {
        refreshInputs(System.nanoTime());
    }

    void refreshInputs(long nowNanos) {
        synchronized (this) {
            if (closed || refreshCycleInFlight) return;
            refreshCycleInFlight = true;
            activeRefreshCycles++;
        }
        try {
            RefreshTask[] tasks;
            synchronized (this) {
                retryBindings(nowNanos);
                int handleCount = motors.size() + encoders.size() + imus.size();
                if (handleCount != refreshTaskHandleCount) {
                    List<RefreshTask> rebuilt = new ArrayList<>(handleCount);
                    motors.forEach((identity, handle) -> rebuilt.add(new RefreshTask(identity, handle::refreshInputs)));
                    encoders.forEach((identity, handle) -> rebuilt.add(new RefreshTask(identity, handle::refreshInputs)));
                    imus.forEach((identity, handle) -> rebuilt.add(new RefreshTask(identity, handle::refreshInputs)));
                    refreshTasks = rebuilt.toArray(RefreshTask[]::new);
                    snapshotSources = snapshotSources();
                    refreshTaskHandleCount = handleCount;
                }
                tasks = refreshTasks;
            }
            for (RefreshTask task : tasks) {
                if (!refresh(task, nowNanos)) return;
            }
            if (motorGroupArray.length > 0) evaluateMotorGroups();
            publishSnapshot(captureSnapshot(nowNanos));
        } finally {
            synchronized (this) {
                refreshCycleInFlight = false;
                activeRefreshCycles--;
                notifyAll();
            }
        }
    }

    private synchronized void evaluateMotorGroups() {
        MotorGroup[] groups = motorGroupArray;
        for (int index = 0; index < groups.length; index++) {
            groups[index].evaluate();
        }
    }

    /** Returns the immutable inputs and health captured by the latest graph refresh. */
    public HardwareCycleSnapshot cycleSnapshot() {
        return cycleSnapshot;
    }

    private HardwareCycleSnapshot captureSnapshot(long nowNanos) {
        SnapshotSources sources;
        long sequence;
        Map<HardwareIdentity, RuntimeException> failures;
        synchronized (this) {
            sources = snapshotSources;
            sequence = ++refreshSequence;
            failures = failureSnapshot(sources);
        }

        // Only one refresh cycle is in flight at a time, so its entry buffers can be reused across cycles.
        if (snapshotEntrySources != sources) {
            motorSnapshotEntries = SnapshotEntry.buffer(sources.motors().length);
            encoderSnapshotEntries = SnapshotEntry.buffer(sources.encoders().length);
            imuSnapshotEntries = SnapshotEntry.buffer(sources.imus().length);
            snapshotEntrySources = sources;
        }
        SnapshotEntry[] motorEntries = motorSnapshotEntries;
        SnapshotEntry[] encoderEntries = encoderSnapshotEntries;
        SnapshotEntry[] imuEntries = imuSnapshotEntries;
        int index = 0;
        for (MotorSnapshotSource source : sources.motors()) {
            MotorHandle handle = source.handle();
            RuntimeException failure = failures.get(source.identity());
            motorEntries[index++].set(source.identity(), new HardwareCycleSnapshot.MotorInput(
                    source.name(),
                    motorPosition(handle), motorVelocity(handle), motorVoltage(handle),
                    motorSupplyCurrent(handle), motorStatorCurrent(handle),
                    failure == null, failureMessage(failure)));
        }
        index = 0;
        for (EncoderSnapshotSource source : sources.encoders()) {
            EncoderHandle handle = source.handle();
            RuntimeException failure = failures.get(source.identity());
            encoderEntries[index++].set(source.identity(), new HardwareCycleSnapshot.EncoderInput(
                    source.name(),
                    encoderPosition(handle), source.absolutePosition(), encoderVelocity(handle),
                    failure == null, failureMessage(failure)));
        }
        index = 0;
        for (ImuSnapshotSource source : sources.imus()) {
            ImuHandle handle = source.handle();
            RuntimeException failure = failures.get(source.identity());
            imuEntries[index++].set(source.identity(), new HardwareCycleSnapshot.ImuInput(
                    source.name(),
                    imuYaw(handle), imuPitch(handle), imuRoll(handle), imuAngle(handle), imuYawRate(handle),
                    imuPitchRate(handle), imuRollRate(handle),
                    imuAccelerationX(handle), imuAccelerationY(handle), imuAccelerationZ(handle),
                    failure == null && imuConnected(handle), imuCalibrating(handle), imuLastUpdate(handle),
                    failureMessage(failure)));
        }
        // Map.ofEntries builds the same immutable map that HardwareCycleSnapshot's Map.copyOf would, and
        // Map.copyOf returns an immutable map as is, so no intermediate map is built.
        Map<HardwareIdentity, HardwareCycleSnapshot.MotorInput> motorInputs = SnapshotEntry.immutableMap(motorEntries);
        Map<HardwareIdentity, HardwareCycleSnapshot.EncoderInput> encoderInputs =
                SnapshotEntry.immutableMap(encoderEntries);
        Map<HardwareIdentity, HardwareCycleSnapshot.ImuInput> imuInputs = SnapshotEntry.immutableMap(imuEntries);
        return new HardwareCycleSnapshot(
                sequence, nowNanos, motorInputs, encoderInputs, imuInputs);
    }

    private SnapshotSources snapshotSources() {
        MotorSnapshotSource[] motorSources = new MotorSnapshotSource[motors.size()];
        EncoderSnapshotSource[] encoderSources = new EncoderSnapshotSource[encoders.size()];
        ImuSnapshotSource[] imuSources = new ImuSnapshotSource[imus.size()];
        int index = 0;
        for (Map.Entry<HardwareIdentity, MotorHandle> entry : motors.entrySet()) {
            // A group leader's snapshot reports its own controller, not the motor leading in its place.
            MotorHandle handle = entry.getValue() instanceof FailoverMotorHandle failover
                    ? failover.original()
                    : entry.getValue();
            motorSources[index++] = new MotorSnapshotSource(
                    entry.getKey(), handle, entry.getValue().device().defaultName());
        }
        index = 0;
        for (Map.Entry<HardwareIdentity, EncoderHandle> entry : encoders.entrySet()) {
            encoderSources[index++] = new EncoderSnapshotSource(
                    entry.getKey(), entry.getValue(), entry.getValue().device().defaultName());
        }
        index = 0;
        for (Map.Entry<HardwareIdentity, ImuHandle> entry : imus.entrySet()) {
            imuSources[index++] = new ImuSnapshotSource(
                    entry.getKey(), entry.getValue(), entry.getValue().device().defaultName());
        }
        return new SnapshotSources(motorSources, encoderSources, imuSources);
    }

    private Map<HardwareIdentity, RuntimeException> failureSnapshot(SnapshotSources sources) {
        if (refreshFailures.isEmpty() && bindingFailures.isEmpty()) return Map.of();
        Map<HardwareIdentity, RuntimeException> failures = new LinkedHashMap<>();
        for (MotorSnapshotSource source : sources.motors()) putFailure(failures, source.identity());
        for (EncoderSnapshotSource source : sources.encoders()) putFailure(failures, source.identity());
        for (ImuSnapshotSource source : sources.imus()) putFailure(failures, source.identity());
        return failures;
    }

    private void putFailure(Map<HardwareIdentity, RuntimeException> failures, HardwareIdentity identity) {
        RuntimeException failure = failure(identity);
        if (failure != null) failures.put(identity, failure);
    }

    private synchronized void publishSnapshot(HardwareCycleSnapshot snapshot) {
        if (!closed && snapshot.sequence() > cycleSnapshot.sequence()) cycleSnapshot = snapshot;
    }

    private static double motorPosition(MotorHandle value) { try { return value.integratedPositionRotations(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double motorVelocity(MotorHandle value) { try { return value.integratedVelocityRotationsPerSecond(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double motorVoltage(MotorHandle value) { try { return value.appliedVoltage(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double motorSupplyCurrent(MotorHandle value) { try { return value.supplyCurrentAmps(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double motorStatorCurrent(MotorHandle value) { try { return value.statorCurrentAmps(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double encoderPosition(EncoderHandle value) { try { return value.positionRotations(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double encoderVelocity(EncoderHandle value) { try { return value.velocityRotationsPerSecond(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double imuYaw(ImuHandle value) { try { return value.yawDegrees(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double imuPitch(ImuHandle value) { try { return value.pitchDegrees(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double imuRoll(ImuHandle value) { try { return value.rollDegrees(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double imuAngle(ImuHandle value) { try { return value.angleDegrees(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double imuYawRate(ImuHandle value) { try { return value.yawRateDegreesPerSecond(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double imuPitchRate(ImuHandle value) { try { return value.pitchRateDegreesPerSecond(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double imuRollRate(ImuHandle value) { try { return value.rollRateDegreesPerSecond(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double imuAccelerationX(ImuHandle value) { try { return value.linearAccelerationXG(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double imuAccelerationY(ImuHandle value) { try { return value.linearAccelerationYG(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static double imuAccelerationZ(ImuHandle value) { try { return value.linearAccelerationZG(); } catch (RuntimeException exception) { return Double.NaN; } }
    private static boolean imuConnected(ImuHandle value) { try { return value.isConnected(); } catch (RuntimeException exception) { return false; } }
    private static boolean imuCalibrating(ImuHandle value) { try { return value.isCalibrating(); } catch (RuntimeException exception) { return false; } }
    private static double imuLastUpdate(ImuHandle value) { try { return value.lastUpdateSeconds(); } catch (RuntimeException exception) { return Double.NaN; } }

    private static String failureMessage(RuntimeException failure) {
        return failure == null || failure.getMessage() == null ? "" : failure.getMessage();
    }

    private RuntimeException failure(HardwareIdentity identity) {
        RuntimeException refreshFailure = refreshFailures.get(identity);
        if (refreshFailure != null) return refreshFailure;
        for (Map.Entry<Object, RuntimeException> entry : bindingFailures.entrySet()) {
            Object declaration = entry.getKey();
            HardwareIdentity declarationIdentity = declaration instanceof MotorDevice motor
                    ? HardwareIdentity.motor(motor)
                    : declaration instanceof EncoderDevice encoder
                            ? HardwareIdentity.encoder(encoder)
                            : declaration instanceof ImuDevice imu ? HardwareIdentity.imu(imu) : null;
            if (identity.equals(declarationIdentity)) return entry.getValue();
        }
        return null;
    }

    /**
     * Retries declarations whose binding failed and whose backoff has elapsed. Runs on the loop thread, so it only
     * queues work for the binder; the vendor calls happen there.
     */
    private void retryBindings(long nowNanos) {
        if (bindingFailures.isEmpty()) return;
        for (Object declaration : new ArrayList<>(bindingFailures.keySet())) {
            if (pendingBindings.containsKey(declaration) || isIdentityPending(identityOf(declaration))) continue;
            RefreshBackoff backoff = bindingRetryBackoffs.get(declaration);
            if (backoff != null && nowNanos < backoff.retryAtNanos()) continue;
            submitBinding(declaration);
        }
    }

    private void submitBinding(Object declaration) {
        HardwareIdentity identity = identityOf(declaration);
        pendingBindings.put(declaration, identity);
        pendingIdentities.merge(identity, 1, Integer::sum);
        try {
            binder().execute(() -> attemptBinding(declaration));
        } catch (RuntimeException exception) {
            pendingBindings.remove(declaration);
            pendingIdentities.computeIfPresent(identity, (key, count) -> count > 1 ? count - 1 : null);
            recordBinderFailure(declaration, exception);
        }
    }

    private ExecutorService binder() {
        if (binderExecutor == null) {
            var executor = new java.util.concurrent.ThreadPoolExecutor(4, 4, 10, TimeUnit.SECONDS,
                    new java.util.concurrent.ArrayBlockingQueue<Runnable>(256), runnable -> {
                Thread thread = new Thread(runnable, "athena-hardware-binder");
                thread.setDaemon(true);
                return thread;
            });
            executor.allowCoreThreadTimeOut(true);
            binderExecutor = executor;
        }
        return binderExecutor;
    }

    /**
     * One background bind attempt. Vendor work runs with the graph lock released; only the bookkeeping takes it. An
     * {@link Error} from a vendor library is recorded as a failure too, so the declaration is not retried every cycle.
     */
    private void attemptBinding(Object declaration) {
        synchronized (this) { runningBindings.add(declaration); }
        try {
            if (declaration instanceof MotorDevice motor) {
                attemptMotor(motor);
            } else if (declaration instanceof EncoderDevice encoder) {
                attemptEncoder(encoder);
            } else {
                attemptImu((ImuDevice) declaration);
            }
        } catch (RuntimeException | Error error) {
            synchronized (this) {
                recordBinderFailure(declaration, error instanceof RuntimeException runtime
                        ? runtime
                        : new IllegalStateException("Hardware binding failed.", error));
            }
        } finally {
            synchronized (this) {
                runningBindings.remove(declaration);
                HardwareIdentity identity = pendingBindings.remove(declaration);
                if (identity != null) {
                    pendingIdentities.computeIfPresent(identity, (key, count) -> count > 1 ? count - 1 : null);
                }
                notifyAll();
            }
        }
    }

    private void attemptMotor(MotorDevice device) {
        PreparedMotor prepared;
        try {
            MotorHandle leader = null;
            synchronized (this) {
                if (closed || (!bindingFailures.containsKey(device) && !pendingBindings.containsKey(device))) return;
                if (motors.containsKey(identity(device))) {
                    createMotor(device);
                    return;
                }
                if (device.follower() != null) {
                    MotorDevice leaderDevice = device.follower().leader();
                    leader = motors.get(identity(leaderDevice));
                    if (leader == null) {
                        throw new DependencyPending(
                                "Motor " + HardwareIdentity.motor(leaderDevice).key() + " is not bound yet.");
                    }
                }
            }
            prepared = prepareMotor(device, leader);
        } catch (RuntimeException exception) {
            synchronized (this) {
                recordBinderFailure(device, exception);
            }
            return;
        }
        boolean release = true;
        synchronized (this) {
            if (!closed) {
                try {
                    installMotor(device, prepared);
                    release = false;
                } catch (RuntimeException exception) {
                    recordBinderFailure(device, exception);
                }
            }
        }
        if (release) closeHandle(prepared.created());
    }

    private void attemptEncoder(EncoderDevice device) {
        EncoderHandle prepared;
        try {
            MotorHandle motor;
            synchronized (this) {
                if (closed || (!bindingFailures.containsKey(device) && !pendingBindings.containsKey(device))) return;
                EncoderHandle existing = encoders.get(identity(device));
                if (existing != null) {
                    bindRuntime(device, existing);
                    clearFailure(device);
                    return;
                }
                motor = boundSourceMotor(device);
            }
            prepared = prepareEncoder(device, motor);
        } catch (RuntimeException exception) {
            synchronized (this) {
                recordBinderFailure(device, exception);
            }
            return;
        }
        boolean release = true;
        synchronized (this) {
            if (!closed) {
                try {
                    installEncoder(device, prepared);
                    release = false;
                } catch (RuntimeException exception) {
                    recordBinderFailure(device, exception);
                }
            }
        }
        if (release) closeHandle(prepared);
    }

    private void attemptImu(ImuDevice device) {
        ImuHandle prepared;
        try {
            synchronized (this) {
                if (closed || (!bindingFailures.containsKey(device) && !pendingBindings.containsKey(device))) return;
                ImuHandle existing = imus.get(identity(device));
                if (existing != null) {
                    bindRuntime(device, existing);
                    clearFailure(device);
                    return;
                }
            }
            prepared = prepareImu(device);
        } catch (RuntimeException exception) {
            synchronized (this) {
                recordBinderFailure(device, exception);
            }
            return;
        }
        boolean release = true;
        synchronized (this) {
            if (!closed) {
                try {
                    installImu(device, prepared);
                    release = false;
                } catch (RuntimeException exception) {
                    recordBinderFailure(device, exception);
                }
            }
        }
        if (release) closeHandle(prepared);
    }

    private HardwareIdentity identityOf(Object declaration) {
        if (declaration instanceof MotorDevice motor) return identity(motor);
        if (declaration instanceof EncoderDevice encoder) return identity(encoder);
        return identity((ImuDevice) declaration);
    }

    private boolean isIdentityPending(HardwareIdentity identity) {
        return pendingIdentities.containsKey(identity);
    }

    /** Records a failed synchronous bind and schedules its retry. */
    private void recordBindingFailure(Object declaration, RuntimeException exception) {
        bindingFailures.put(declaration, exception);
        bindingRetryBackoffs.put(declaration, nextBackoff(declaration));
    }

    /**
     * Records a background bind failure. It is also queued for the loop's operation-failure drain, which reports it
     * once per backoff step (the binder runs once per step, never once per cycle).
     */
    private void recordBinderFailure(Object declaration, RuntimeException exception) {
        if (closed) return;
        recordBindingFailure(declaration, exception);
        if (!(exception instanceof DependencyPending)) operationFailures.put(declaration, exception);
    }

    private RefreshBackoff nextBackoff(Object declaration) {
        RefreshBackoff previous = bindingRetryBackoffs.get(declaration);
        int failureCount = previous == null ? 1 : previous.failureCount() + 1;
        return new RefreshBackoff(failureCount, clock.getAsLong() + retryDelayMillis(failureCount) * 1_000_000L);
    }

    private void clearFailure(Object declaration) {
        bindingFailures.remove(declaration);
        bindingRetryBackoffs.remove(declaration);
        placeholders.remove(declaration);
    }

    /** Closes a handle that was created but will not be published. Its close failure cannot be acted on here. */
    private static void closeHandle(Object handle) {
        if (handle instanceof AutoCloseable closeable) {
            try {
                closeable.close();
            } catch (Exception ignored) {
                // The handle was never published, so there is nothing else to undo.
            }
        }
    }

    /** Test hook: waits until no background binding is queued or running. Returns false on timeout. */
    synchronized boolean awaitBindingsIdle(long timeoutMillis) throws InterruptedException {
        long deadline = System.nanoTime() + TimeUnit.MILLISECONDS.toNanos(timeoutMillis);
        while (!pendingBindings.isEmpty()) {
            long remaining = deadline - System.nanoTime();
            if (remaining <= 0) return false;
            TimeUnit.NANOSECONDS.timedWait(this, remaining);
        }
        return true;
    }

    /** Replaces the clock that backoff deadlines are measured on. Tests use it; production keeps System.nanoTime. */
    synchronized void setBindingClock(LongSupplier source) {
        clock = Objects.requireNonNull(source, "source");
    }

    /**
     * Returns failures recorded during the latest input refresh.
     *
     * @return refresh failures
     */
    public synchronized List<RefreshFailure> refreshFailures() {
        if (refreshFailureSnapshotDirty) {
            refreshFailureSnapshot = refreshFailures.entrySet().stream()
                    .map(entry -> new RefreshFailure(
                            entry.getKey(), declaration(entry.getKey()), entry.getValue()))
                    .toList();
            refreshFailureSnapshotDirty = false;
        }
        return refreshFailureSnapshot;
    }

    /** Returns declarations that could not be bound during graph construction. */
    public synchronized List<BindingFailure> bindingFailures() {
        return bindingFailures.entrySet().stream()
                .filter(entry -> !(entry.getValue() instanceof DependencyPending))
                .map(entry -> new BindingFailure(entry.getKey(), entry.getValue()))
                .toList();
    }

    @Override
    public synchronized void hardwareFailure(Object declaration, RuntimeException exception) {
        operationFailures.put(
                Objects.requireNonNull(declaration, "declaration"),
                Objects.requireNonNull(exception, "exception"));
        if (declaration instanceof MotorDevice motor && motorGroupArray.length > 0) {
            HardwareIdentity identity = identity(motor);
            for (MotorGroup group : motorGroupArray) {
                if (group.contains(identity)) group.operationFailed(identity);
            }
        }
    }

    // The same empty list the stream below produces, returned without building a stream when nothing failed.
    private static final List<BindingFailure> NO_OPERATION_FAILURES =
            java.util.stream.Stream.<BindingFailure>empty().toList();

    /** Returns and clears output/setup failures reported since the previous drain. */
    public synchronized List<BindingFailure> drainOperationFailures() {
        if (operationFailures.isEmpty()) return NO_OPERATION_FAILURES;
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

    private boolean refresh(RefreshTask task, long nowNanos) {
        synchronized (this) {
            if (closed || task.inFlight) return false;
            RefreshBackoff backoff = refreshBackoffs.get(task.identity);
            if (backoff != null && nowNanos < backoff.retryAtNanos()) return true;
            task.inFlight = true;
        }

        RuntimeException failure = null;
        boolean completed = false;
        try {
            task.refresh.run();
            completed = true;
        } catch (RuntimeException exception) {
            failure = exception;
            completed = true;
        } finally {
            synchronized (this) {
                task.inFlight = false;
                if (completed) recordRefreshResult(task.identity, nowNanos, failure);
            }
        }
        return true;
    }

    private void recordRefreshResult(HardwareIdentity identity, long nowNanos, RuntimeException failure) {
        RefreshBackoff backoff = refreshBackoffs.get(identity);
        if (failure == null) {
            if (refreshFailures.remove(identity) != null) refreshFailureSnapshotDirty = true;
            refreshBackoffs.remove(identity);
            return;
        }
        int failureCount = backoff == null ? 1 : backoff.failureCount() + 1;
        refreshFailures.put(identity, failure);
        refreshFailureSnapshotDirty = true;
        refreshBackoffs.put(identity, new RefreshBackoff(
                failureCount,
                nowNanos + retryDelayMillis(failureCount) * 1_000_000L));
    }

    private static long retryDelayMillis(int failureCount) {
        return Math.min(2_000L, 100L << Math.min(failureCount - 1, 4));
    }

    /** Explicit shutdown; may wait up to one second for binding workers, never from robot periodic. */
    @Override
    public void close() {
        ExecutorService binder;
        List<Object> releases = new ArrayList<>();
        synchronized (this) {
            if (closed) return;
            closed = true;
            binder = binderExecutor;
            binderExecutor = null;
            while (activeRefreshCycles > 0) {
                try {
                    wait();
                } catch (InterruptedException exception) {
                    Thread.currentThread().interrupt();
                    throw new IllegalStateException("Interrupted while waiting for hardware refresh to finish.", exception);
                }
            }
            releases.addAll(runtimeBindings.values());
            runtimeBindings.clear();
            runtimeBindingHandles.clear();
            motorLookups.clear();
            motorEpoch++;
            releases.addAll(imus.values());
            releases.addAll(encoders.values());
            releases.addAll(motors.values());
            imus.clear();
            encoders.clear();
            motors.clear();
            softwareFollowers.clear();
            softwareFollowerViews.clear();
            followerConstraintViews.clear();
            motorGroups.clear();
            motorGroupArray = new MotorGroup[0];
            motorIdentities.clear();
            encoderIdentities.clear();
            imuIdentities.clear();
            refreshFailures.clear();
            refreshFailureSnapshot = List.of();
            refreshFailureSnapshotDirty = false;
            refreshBackoffs.clear();
            bindingRetryBackoffs.clear();
            bindingFailures.clear();
            operationFailures.clear();
            placeholders.clear();
            refreshTasks = new RefreshTask[0];
            refreshTaskHandleCount = -1;
            cycleSnapshot = HardwareCycleSnapshot.empty();
        }
        // Native cleanup must never own the graph monitor. Vendor queues fence in-flight work first.
        for (Object handle : releases) closeHandle(handle);
        if (binder != null) {
            stopBinder(binder);
            synchronized (this) {
                // shutdownNow removes queued lambdas without their finally blocks. Active prepares
                // retain accounting until they exit; discarded queued descriptors are removed here.
                var pending = pendingBindings.entrySet().iterator();
                while (pending.hasNext()) {
                    var entry = pending.next();
                    if (runningBindings.contains(entry.getKey())) continue;
                    pendingIdentities.computeIfPresent(entry.getValue(), (key, count) -> count > 1 ? count - 1 : null);
                    pending.remove();
                }
                notifyAll();
            }
        }
    }

    /**
     * Stops the binder without waiting on a vendor call that never returns: the thread is a daemon, and any binding
     * that finishes after close is discarded because {@link #closed} is set.
     */
    private static void stopBinder(ExecutorService binder) {
        binder.shutdown();
        try {
            if (!binder.awaitTermination(1, TimeUnit.SECONDS)) {
                binder.shutdownNow();
            }
        } catch (InterruptedException exception) {
            Thread.currentThread().interrupt();
        }
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

    private HardwareIdentity identity(MotorDevice device) {
        return motorIdentities.computeIfAbsent(device, HardwareIdentity::motor);
    }

    private HardwareIdentity identity(EncoderDevice device) {
        return encoderIdentities.computeIfAbsent(device, HardwareIdentity::encoder);
    }

    private HardwareIdentity identity(ImuDevice device) {
        return imuIdentities.computeIfAbsent(device, HardwareIdentity::imu);
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
        @Override public ca.frc6390.athena.hardware.backend.ConfigurationStatus configurationStatus() {
            return motor.configurationStatus();
        }
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
        @Override public ca.frc6390.athena.hardware.backend.ConfigurationStatus configurationStatus() {
            return delegate.configurationStatus();
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
        @Override public ca.frc6390.athena.hardware.backend.ConfigurationStatus configurationStatus() {
            return motor.configurationStatus();
        }
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

    private static final class RefreshTask {
        private final HardwareIdentity identity;
        private final Runnable refresh;
        private boolean inFlight;

        private RefreshTask(HardwareIdentity identity, Runnable refresh) {
            this.identity = Objects.requireNonNull(identity, "identity");
            this.refresh = Objects.requireNonNull(refresh, "refresh");
        }
    }

    /**
     * Reusable map entry handed to {@link Map#ofEntries}, which copies keys and values and keeps no entry.
     */
    private static final class SnapshotEntry implements Map.Entry<HardwareIdentity, Object> {
        private static final SnapshotEntry[] EMPTY = new SnapshotEntry[0];
        private HardwareIdentity key;
        private Object value;

        private static SnapshotEntry[] buffer(int size) {
            if (size == 0) return EMPTY;
            SnapshotEntry[] entries = new SnapshotEntry[size];
            for (int i = 0; i < size; i++) entries[i] = new SnapshotEntry();
            return entries;
        }

        @SuppressWarnings("unchecked")
        private static <V> Map<HardwareIdentity, V> immutableMap(SnapshotEntry[] entries) {
            Map<HardwareIdentity, V> map = (Map<HardwareIdentity, V>) Map.ofEntries(entries);
            for (SnapshotEntry entry : entries) entry.value = null;
            return map;
        }

        private void set(HardwareIdentity key, Object value) {
            this.key = key;
            this.value = value;
        }

        @Override public HardwareIdentity getKey() { return key; }
        @Override public Object getValue() { return value; }
        @Override public Object setValue(Object value) { throw new UnsupportedOperationException(); }
    }

    private record SnapshotSources(
            MotorSnapshotSource[] motors,
            EncoderSnapshotSource[] encoders,
            ImuSnapshotSource[] imus) {
        private static SnapshotSources empty() {
            return new SnapshotSources(
                    new MotorSnapshotSource[0], new EncoderSnapshotSource[0], new ImuSnapshotSource[0]);
        }
    }

    private record MotorSnapshotSource(
            HardwareIdentity identity, MotorHandle handle, String name) {
    }

    private static final class EncoderSnapshotSource {
        private final HardwareIdentity identity;
        private final EncoderHandle handle;
        private final String name;
        private boolean absoluteSupported = true;

        private EncoderSnapshotSource(HardwareIdentity identity, EncoderHandle handle, String name) {
            this.identity = identity;
            this.handle = handle;
            this.name = name;
        }

        private HardwareIdentity identity() { return identity; }
        private EncoderHandle handle() { return handle; }
        private String name() { return name; }

        private double absolutePosition() {
            if (!absoluteSupported) return Double.NaN;
            try {
                return handle.absolutePositionRotations();
            } catch (UnsupportedOperationException exception) {
                absoluteSupported = false;
                return Double.NaN;
            } catch (RuntimeException exception) {
                return Double.NaN;
            }
        }
    }

    private record ImuSnapshotSource(
            HardwareIdentity identity, ImuHandle handle, String name) {
    }

    /** A motor created off the graph lock, with its vendor handle kept for disposal if it is not published. */
    private record PreparedMotor(MotorHandle created, MotorHandle handle, boolean softwareFollow) {
    }

    private record RefreshBackoff(int failureCount, long retryAtNanos) {
    }

    /** A queued dependency is not a hardware fault; retry with backoff without applying failure policy. */
    private static final class DependencyPending extends RuntimeException {
        private static final long serialVersionUID = 1L;
        private DependencyPending(String message) { super(message); }
    }

    @FunctionalInterface
    private interface RuntimeBindingFactory {
        AutoCloseable bind();
    }

    private final class GroupWiring implements MotorGroup.Wiring {
        @Override public void constraintsChanged() { followerRevision++; }
        @Override
        public RuntimeException refreshFailure(HardwareIdentity identity) {
            return refreshFailures.isEmpty() ? null : refreshFailures.get(identity);
        }

        @Override
        public boolean supportsHardwareFollowing(MotorDevice follower, MotorDevice leader) {
            return backends.motorBackendFor(follower.kind())
                    .map(backend -> backend.supportsHardwareFollowing(follower, leader))
                    .orElse(false);
        }

        @Override
        public void softwareFollower(ActionContext.SoftwareMotorFollower follower, boolean add) {
            // The group's motors are mirrored from the declared leader, whichever motor leads it.
            for (MotorGroup group : motorGroupArray) {
                if (!group.contains(identity(follower.device()))) continue;
                HardwareIdentity leaderIdentity = group.leaderIdentity();
                List<ActionContext.SoftwareMotorFollower> followers =
                        softwareFollowers.computeIfAbsent(leaderIdentity, ignored -> new ArrayList<>());
                if (add) {
                    followers.add(follower);
                } else {
                    followers.removeIf(existing -> existing == follower);
                }
                softwareFollowerViews.remove(leaderIdentity);
                return;
            }
        }

        @Override
        public boolean absoluteEncoderOn(HardwareIdentity motor) {
            for (EncoderHandle encoder : encoders.values()) {
                if (encoder.device().source() instanceof EncoderDevice.EncoderSource.MotorAbsolute absolute
                        && identity(absolute.motor()).equals(motor)) {
                    return true;
                }
            }
            return false;
        }
    }

    private record UnavailableMotorHandle(MotorDevice device) implements MotorHandle {
        @Override public ca.frc6390.athena.hardware.backend.ConfigurationStatus configurationStatus() {
            return new ca.frc6390.athena.hardware.backend.ConfigurationStatus(1, 0, true, "");
        }
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
        @Override public ca.frc6390.athena.hardware.backend.ConfigurationStatus configurationStatus() {
            return new ca.frc6390.athena.hardware.backend.ConfigurationStatus(1, 0, true, "");
        }
        @Override public double positionRotations() { return 0.0; }
        @Override public double absolutePositionRotations() { return 0.0; }
        @Override public double velocityRotationsPerSecond() { return 0.0; }
        @Override public void setPositionRotations(double rotations) { }
        @Override public boolean supportsPositionSetting() { return true; }
    }

    private record UnavailableImuHandle(ImuDevice device) implements ImuHandle {
        @Override public ca.frc6390.athena.hardware.backend.ConfigurationStatus configurationStatus() {
            return new ca.frc6390.athena.hardware.backend.ConfigurationStatus(1, 0, true, "");
        }
        @Override public boolean isConnected() { return false; }
        @Override public boolean isCalibrating() { return true; }
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
