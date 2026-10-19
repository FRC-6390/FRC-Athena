package ca.frc6390.athena.mechanism.core;

import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.units.measure.Voltage;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.MappedActionContext;
import ca.frc6390.athena.hardware.power.PowerPolicy;
import ca.frc6390.athena.hardware.power.PowerSnapshot;
import ca.frc6390.athena.mechanism.staging.StageCaps;
import ca.frc6390.athena.mechanism.staging.StageHost;
import ca.frc6390.athena.mechanism.staging.StageModifier;
import ca.frc6390.athena.mechanism.staging.Staging;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import ca.frc6390.athena.mechanism.model.Characterization;
import ca.frc6390.athena.mechanism.persist.PersistentModelStorage;
import ca.frc6390.athena.mechanism.persist.PersistentStore;
import ca.frc6390.athena.mechanism.persist.TelemetryPersistence;
import ca.frc6390.athena.mechanism.persist.TuningLocks;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

/**
 * Mechanism scheduler used by the root robot runtime.
 */
public final class MechanismScheduler {
    private static final int COMPLETION_HISTORY_LIMIT = 256;
    private final MappedActionContext registeredHandles = new MappedActionContext();
    private final ActionContext actionContext;
    private final Map<Mechanism, MechanismRuntime> runtimes = new IdentityHashMap<>();
    private final Map<Action, RequestTarget> actionTargets = new IdentityHashMap<>();
    private final Map<Action, List<String>> actionDeclarationLocations = new IdentityHashMap<>();
    private final Map<Action, CompiledAction> compiledActions = new IdentityHashMap<>();
    private final Set<Action> ambiguousActionTargets = Collections.newSetFromMap(new IdentityHashMap<>());
    private final Map<Object, RequestTarget> declarationTargets = new LinkedHashMap<>();
    private final Set<Object> ambiguousDeclarationTargets = new LinkedHashSet<>();
    private final Set<DigitalInputDevice> digitalInputs = new LinkedHashSet<>();
    private final RobotGraph graph = new RobotGraph();
    private final Map<PathAction, PathRuntime> pathRuntimes = new LinkedHashMap<>();
    private final Map<Object, List<LeaseRegistration>> leaseTargets = new IdentityHashMap<>();
    private final Map<Object, LeaseReservation> leaseReservations = new IdentityHashMap<>();
    private final List<Object> activeLeaseKeys = new ArrayList<>();
    private final List<Object> completedLeaseKeys = new ArrayList<>();
    private final List<Object> retirementScratch = new ArrayList<>();
    private final Map<Object, Long> reservationScratch = new IdentityHashMap<>();
    private final Map<MechanismRuntime, Set<MotorDevice>> drivenByRuntimeScratch = new IdentityHashMap<>();
    private final Set<MotorDevice> globallyDrivenScratch = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    private final Map<Object, Set<MotorDevice>> faultSuppressions = new IdentityHashMap<>();
    private final Map<MotorDevice, Integer> faultSuppressionCounts = new IdentityHashMap<>();
    private final Map<Action, Set<Object>> liveResourceCache = new IdentityHashMap<>();
    private final Map<PathAction, Set<Object>> pathOwnershipCache = new IdentityHashMap<>();
    private final Map<Object, Set<Action>> spawnedLeases = new IdentityHashMap<>();
    private final List<Object> evaluatingLeases = new ArrayList<>();
    /** Interrupt policy of each running root Action; absent means ANYTIME. */
    private final Map<Object, Interrupt> leaseInterrupts = new IdentityHashMap<>();
    /** Newer requests held because a running Action does not allow interruption yet. */
    private final List<PendingRequest> pendingRequests = new ArrayList<>();
    private final List<Mechanism> pendingAborts = new ArrayList<>();
    private final List<Action> deferredRequests = new ArrayList<>();
    private final List<RuntimeFault> pendingFaults = new ArrayList<>();
    private final Map<String, FaultHistory> faultHistory = new LinkedHashMap<>();
    private static final double FAULT_REPORT_PERIOD_SECONDS = 5.0;
    private boolean isolateFaults;
    private double cycleSeconds;
    private final MechanismRuntime.LeaseResources leaseResources = new MechanismRuntime.LeaseResources() {
        @Override
        public Set<Object> of(Action action) {
            Set<Object> cached = liveResourceCache.get(action);
            if (cached == null) {
                cached = actionResources(action);
                liveResourceCache.put(action, cached);
            }
            return cached;
        }

        @Override
        public Set<Object> pathOwnership(PathAction path) {
            Set<Object> cached = pathOwnershipCache.get(path);
            if (cached == null) {
                Set<Object> resources = new LinkedHashSet<>();
                PathRuntime runtime = pathRuntimes.get(path);
                if (runtime != null) runtime.ownership(path).forEach(value -> collectOwnedResource(value, resources));
                cached = Set.copyOf(resources);
                pathOwnershipCache.put(path, cached);
            }
            return cached;
        }
    };
    private final MechanismRuntime.RuntimeHooks runtimeHooks = new MechanismRuntime.RuntimeHooks() {
        @Override
        public void enterLease(Object owner) {
            evaluatingLeases.add(owner);
        }

        @Override
        public void exitLease() {
            if (!evaluatingLeases.isEmpty()) evaluatingLeases.remove(evaluatingLeases.size() - 1);
        }

        @Override
        public void fault(String location, RuntimeException exception) {
            recordFault(location, exception);
        }
    };
    private final OutputResolver resolver;
    private final PowerRuntime power = new PowerRuntime();
    private final List<Staging<?>> stagings = new ArrayList<>();
    private final List<Characterization> characterizations = new ArrayList<>();
    private List<MotorDevice> powerMotors = List.of();
    private final StageHost stageHost = new StageHost() {
        @Override public void request(Action action) { MechanismScheduler.this.request(action); }
        @Override public void cancel(Action action) { MechanismScheduler.this.cancel(action); }
        @Override public boolean isRunning(Action action) { return MechanismScheduler.this.isRunning(action); }
    };
    private long requestSequence;
    private Runnable simulationStep = () -> {
    };
    private volatile TelemetrySchema telemetrySchema;
    private MechanismTraceLevel traceLevel = MechanismTraceLevel.OFF;
    private double tracePeriodSeconds = 0.10;
    private int outputCapacityHint;

    private MechanismScheduler(OutputResolver resolver, ActionContext hardwareContext) {
        this.resolver = Objects.requireNonNull(resolver, "resolver");
        actionContext = new OverlayActionContext(registeredHandles,
                hardwareContext == null ? ActionContext.empty() : hardwareContext);
        resolver.overrides().actionContext(actionContext);
        resolver.overrides().electrical(power);
    }

    public static MechanismScheduler create() {
        return new MechanismScheduler(OutputResolver.empty(), ActionContext.empty());
    }

    static MechanismScheduler create(OutputResolver resolver) {
        return new MechanismScheduler(resolver, ActionContext.empty());
    }

    /**
     * Creates a runtime backed by a runtime-owned hardware context.
     *
     * @param hardwareContext hardware context
     * @return mechanism scheduler
     */
    public static MechanismScheduler create(ActionContext hardwareContext) {
        return new MechanismScheduler(OutputResolver.empty(), hardwareContext);
    }

    /**
     * Creates a runtime backed by an output resolver and hardware context.
     *
     * @param resolver output resolver
     * @param hardwareContext hardware context
     * @return mechanism scheduler
     */
    static MechanismScheduler create(OutputResolver resolver, ActionContext hardwareContext) {
        return new MechanismScheduler(resolver, hardwareContext);
    }

    public MechanismScheduler register(Mechanism mechanism) {
        Objects.requireNonNull(mechanism, "mechanism");
        if (graph.contains(runtimes.keySet(), mechanism)) {
            return this;
        }
        graph.node(mechanism);
        runtimes.computeIfAbsent(mechanism, this::runtime);
        runtimes.get(mechanism).traceLevel(traceLevel);
        runtimes.get(mechanism).tracePeriodSeconds(tracePeriodSeconds);
        indexActionTargets(
                mechanism,
                mechanism,
                graph.node(mechanism).name(),
                Collections.newSetFromMap(new IdentityHashMap<>()));
        indexDeclarationTargets(mechanism, mechanism, Collections.newSetFromMap(new IdentityHashMap<>()));
        refreshPowerDeclarations();
        attachCharacterizations(mechanism);
        rebuildCompiledActions();
        refreshDigitalInputs();
        telemetrySchema = null;
        return this;
    }

    public MechanismScheduler motor(MotorDevice device, MotorHandle motor) {
        MotorDevice safeDevice = Objects.requireNonNull(device, "device");
        registeredHandles.motor(safeDevice, Objects.requireNonNull(motor, "motor"));
        return this;
    }

    public MechanismScheduler encoder(EncoderDevice device, EncoderHandle encoder) {
        registeredHandles.encoder(Objects.requireNonNull(device, "device"), Objects.requireNonNull(encoder, "encoder"));
        return this;
    }

    public MechanismScheduler imu(ImuDevice device, ImuHandle imu) {
        registeredHandles.imu(Objects.requireNonNull(device, "device"), Objects.requireNonNull(imu, "imu"));
        return this;
    }

    public MechanismScheduler digitalInput(DigitalInputDevice device, BooleanSupplier reader) {
        DigitalInputDevice safeDevice = Objects.requireNonNull(device, "device");
        DigitalInputDevice.bindRuntime(safeDevice, Objects.requireNonNull(reader, "reader"));
        return this;
    }

    public MechanismScheduler path(PathAction path, PathRuntime runtime) {
        pathRuntimes.put(Objects.requireNonNull(path, "path"), Objects.requireNonNull(runtime, "runtime"));
        compiledActions.clear();
        liveResourceCache.clear();
        pathOwnershipCache.clear();
        return this;
    }

    public MechanismScheduler paths(Object root, Function<PathAction, PathRuntime> runtimeFactory) {
        Objects.requireNonNull(runtimeFactory, "runtimeFactory");
        if (runtimes.size() != 1) {
            throw new IllegalStateException("Path declarations require exactly one registered mechanism root.");
        }
        Mechanism owner = runtimes.keySet().iterator().next();
        RequestTarget target = new RequestTarget(owner, owner);
        for (PathAction path : PathIntrospector.inspect(root)) {
            path(path, runtimeFactory.apply(path));
            indexDeclarationTarget(path, target);
        }
        return this;
    }

    public MechanismScheduler path(PathAction path, double seconds) {
        return path(path, PathRuntime.timed(seconds));
    }

    public MechanismScheduler simulationStep(Runnable simulationStep) {
        this.simulationStep = simulationStep == null ? () -> {
        } : simulationStep;
        for (MechanismRuntime runtime : runtimes.values()) {
            runtime.simulationStep(this.simulationStep);
        }
        return this;
    }

    public MechanismScheduler bindInMemoryRuntime() {
        Set<Object> declarations = graph.declarations(runtimes.keySet());
        for (Object declaration : declarations) {
            if (declaration instanceof MotorDevice motor && !canResolveMotor(motor)) {
                MemoryMotor memory = new MemoryMotor(motor);
                motor(motor, memory);
            } else if (declaration instanceof EncoderDevice encoder && !canResolveEncoder(encoder)) {
                MemoryEncoder memory = new MemoryEncoder(encoder);
                encoder(encoder, memory);
            } else if (declaration instanceof DigitalInputDevice digital) {
                DigitalInputDevice.bindRuntime(digital, () -> false);
            }
        }
        return this;
    }

    public ActionContext actionContext() {
        return actionContext;
    }

    /**
     * Samples declared signal inputs once for the current runtime cycle.
     *
     * @return this runtime
     */
    public MechanismScheduler sampleSignals() {
        digitalInputs.forEach(DigitalInputDevice::sample);
        return this;
    }

    /**
     * Returns digital input declarations owned by registered mechanisms.
     *
     * @return declared digital inputs
     */
    public Set<DigitalInputDevice> digitalInputDevices() {
        return Set.copyOf(digitalInputs);
    }

    public List<Mechanism> mechanisms() {
        return List.copyOf(runtimes.keySet());
    }

    /** Selects how much trace detail runtimes materialize each cycle. */
    public MechanismScheduler traceLevel(MechanismTraceLevel level) {
        traceLevel = Objects.requireNonNull(level, "level");
        for (MechanismRuntime runtime : runtimes.values()) runtime.traceLevel(level);
        return this;
    }

    /** Sets the minimum interval between materialized trace snapshots. Zero captures every cycle. */
    public MechanismScheduler tracePeriodSeconds(double periodSeconds) {
        if (!Double.isFinite(periodSeconds) || periodSeconds < 0.0) {
            throw new IllegalArgumentException("Trace period must be finite and non-negative.");
        }
        tracePeriodSeconds = periodSeconds;
        for (MechanismRuntime runtime : runtimes.values()) runtime.tracePeriodSeconds(periodSeconds);
        return this;
    }

    /** Associates a runtime-discovered declaration with a registered mechanism. */
    public MechanismScheduler declarationOwner(Object declaration, Mechanism owner) {
        RequestTarget target = declarationTargets.get(Objects.requireNonNull(owner, "owner"));
        if (target == null) throw new IllegalArgumentException("Mechanism is not registered: " + owner.getClass().getName());
        indexDeclarationTarget(Objects.requireNonNull(declaration, "declaration"), target);
        return this;
    }

    /** Gives a rebound runtime declaration the same owner as its source declaration. */
    public MechanismScheduler declarationAlias(Object declaration, Object source) {
        RequestTarget target = declarationTargets.get(Objects.requireNonNull(source, "source"));
        if (target != null) indexDeclarationTarget(Objects.requireNonNull(declaration, "declaration"), target);
        return this;
    }

    public MechanismTraceLevel traceLevel() { return traceLevel; }

    /** Returns the minimum interval between materialized trace snapshots. */
    public double tracePeriodSeconds() { return tracePeriodSeconds; }

    /** Suppresses one failed declaration and neutralizes it when it is a motor. */
    public void disableDeclaration(Object declaration) {
        if (declaration instanceof MotorDevice motor) {
            suppressFault(declaration, Set.of(motor));
            try {
                actionContext.motor(motor).stop();
            } catch (RuntimeException ignored) {
                // A failed device may be unable to accept the neutral command.
            }
        } else if (declaration instanceof ControlBinding<?> control) {
            suppressFault(declaration, new LinkedHashSet<>(control.motors()));
        }
    }

    /** Suppresses every output owned by the mechanism that declares the failed object. */
    public boolean disableOwner(Object declaration) {
        RequestTarget target = declarationTargets.get(declaration);
        if (target == null) {
            return false;
        }
        Set<MotorDevice> motors = new LinkedHashSet<>();
        for (Object owned : graph.declarations(List.of(target.owner()))) {
            if (owned instanceof MotorDevice motor) motors.add(motor);
            else if (owned instanceof ControlBinding<?> control) motors.addAll(control.motors());
        }
        suppressFault(declaration, motors);
        return true;
    }

    /** Clears the transient suppression created by a recovered declaration. */
    public void recoverFailure(Object declaration) {
        Set<MotorDevice> motors = faultSuppressions.remove(declaration);
        if (motors == null) return;
        for (MotorDevice motor : motors) {
            int remaining = faultSuppressionCounts.getOrDefault(motor, 0) - 1;
            if (remaining <= 0) {
                faultSuppressionCounts.remove(motor);
                resolver.overrides().faulted(motor, false);
            } else {
                faultSuppressionCounts.put(motor, remaining);
            }
        }
    }

    private void suppressFault(Object declaration, Set<MotorDevice> motors) {
        if (motors.isEmpty() || faultSuppressions.containsKey(declaration)) return;
        Set<MotorDevice> snapshot = Set.copyOf(motors);
        faultSuppressions.put(declaration, snapshot);
        for (MotorDevice motor : snapshot) {
            faultSuppressionCounts.merge(motor, 1, Integer::sum);
            resolver.overrides().faulted(motor, true);
        }
    }

    /** Returns the hierarchical, mechanism-scoped telemetry contract. */
    public TelemetrySchema telemetrySchema() {
        TelemetrySchema cached = telemetrySchema;
        if (cached == null) {
            cached = graph.telemetrySchema(runtimes.keySet(), resolver.overrides(), this);
            telemetrySchema = cached;
        }
        return cached;
    }

    /**
     * Returns the latest execution traces without refreshing hardware.
     *
     * @return latest mechanism traces
     */
    public List<MechanismTraceSnapshot> traceSnapshots() {
        if (traceLevel == MechanismTraceLevel.OFF) return List.of();
        List<MechanismTraceSnapshot> traces = new ArrayList<>();
        runtimes.forEach((mechanism, runtime) -> {
            MechanismTraceSnapshot root = runtime.traceSnapshot();
            traces.add(root);
            addChildTraces(traces, mechanism, root.mechanism(), root, "");
        });
        return List.copyOf(traces);
    }

    private void addChildTraces(
            List<MechanismTraceSnapshot> traces,
            Mechanism parent,
            String parentPath,
            MechanismTraceSnapshot root,
            String hookPrefix) {
        graph.node(parent).children().forEach((fieldName, child) -> {
            String path = parentPath + "/" + fieldName;
            String childHookPrefix = hookPrefix + fieldName + ".";
            traces.add(childTrace(path, child, childHookPrefix, root));
            addChildTraces(traces, child, path, root, childHookPrefix);
        });
    }

    private MechanismTraceSnapshot childTrace(
            String path,
            Mechanism mechanism,
            String hookPrefix,
            MechanismTraceSnapshot root) {
        if (traceLevel == MechanismTraceLevel.SUMMARY) {
            return new MechanismTraceSnapshot(
                    path, root.timestampSeconds(), root.timeInStateSeconds(), root.enabled(),
                    root.requestedAction(), root.requestedActionType(), root.scheduledActionType(),
                    root.schedulerStep(), root.schedulerComplete(), root.activeLeaseCount(),
                    List.of(), List.of(), List.of(), List.of());
        }
        Set<String> motorNames = new LinkedHashSet<>();
        for (Object declaration : graph.declarations(List.of(mechanism))) {
            if (declaration instanceof MotorDevice motor) {
                motorNames.add(motor.defaultName());
            }
        }
        List<MechanismTraceSnapshot.ActionCandidate> candidates = root.candidates().stream()
                .filter(candidate -> candidate.motors().stream().anyMatch(motorNames::contains))
                .toList();
        List<MechanismTraceSnapshot.Control> controls = root.controls().stream()
                .filter(control -> motorNames.contains(control.name()))
                .toList();
        List<MechanismTraceSnapshot.Motor> motors = root.motors().stream()
                .filter(motor -> motorNames.contains(motor.name()))
                .toList();
        List<MechanismTraceSnapshot.Hook> hooks = root.hooks().stream()
                .filter(hook -> hook.name().startsWith(hookPrefix))
                .map(hook -> new MechanismTraceSnapshot.Hook(
                        hook.name().substring(hookPrefix.length()),
                        hook.sourceActive(),
                        hook.active(),
                        hook.triggeredThisCycle()))
                .toList();
        return new MechanismTraceSnapshot(
                path,
                root.timestampSeconds(),
                root.timeInStateSeconds(),
                root.enabled(),
                root.requestedAction(),
                root.requestedActionType(),
                root.scheduledActionType(),
                root.schedulerStep(),
                root.schedulerComplete(),
                root.activeLeaseCount(),
                candidates,
                controls,
                motors,
                hooks);
    }

    /**
     * Returns simulation models declared by registered mechanisms.
     *
     * @return simulation models
     */
    public Set<SimModel> simulationModels() {
        return graph.simulations(runtimes.keySet());
    }

    /**
     * Returns control bindings available for automatic functional simulation.
     *
     * @return declared controls
     */
    public Set<ControlBinding<?>> controlBindings() {
        Set<ControlBinding<?>> controls = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
        for (Object declaration : graph.declarations(runtimes.keySet())) {
            if (declaration instanceof ControlBinding<?> control) {
                controls.add(control);
            }
        }
        return java.util.Collections.unmodifiableSet(controls);
    }

    /**
     * Returns IMU declarations owned by registered mechanisms.
     *
     * @return declared IMUs
     */
    public Set<ImuDevice> imuDevices() {
        Set<ImuDevice> imus = new LinkedHashSet<>();
        for (Object declaration : graph.declarations(runtimes.keySet())) {
            if (declaration instanceof ImuDevice imu) {
                imus.add(imu);
            }
        }
        return Set.copyOf(imus);
    }

    /** Returns every encoder declaration required by the registered mechanism graph. */
    public Set<EncoderDevice> encoderDevices() {
        Set<EncoderDevice> encoders = new LinkedHashSet<>();
        for (Object declaration : graph.declarations(runtimes.keySet())) {
            collectEncoders(declaration, encoders);
        }
        return Set.copyOf(encoders);
    }

    /** Returns every motor declaration required by the registered mechanism graph. */
    public Set<MotorDevice> motorDevices() {
        Set<MotorDevice> motors = new LinkedHashSet<>();
        for (Object declaration : graph.declarations(runtimes.keySet())) {
            collectMotors(declaration, motors);
        }
        return Set.copyOf(motors);
    }

    private static void collectMotors(Object declaration, Set<MotorDevice> motors) {
        if (declaration instanceof MotorDevice motor) {
            if (motor.follower() != null) {
                collectMotors(motor.follower().leader(), motors);
            }
            motors.add(motor);
        } else if (declaration instanceof ControlBinding<?> control) {
            control.motors().forEach(motor -> collectMotors(motor, motors));
        } else if (declaration instanceof Iterable<?> values) {
            values.forEach(value -> collectMotors(value, motors));
        }
    }

    private static void collectEncoders(Object declaration, Set<EncoderDevice> encoders) {
        if (declaration instanceof EncoderDevice encoder) {
            encoders.add(encoder);
        } else if (declaration instanceof ControlBinding<?> control) {
            controlDeclarations(control).forEach(value -> collectEncoders(value, encoders));
        } else if (declaration instanceof Iterable<?> values) {
            values.forEach(value -> collectEncoders(value, encoders));
        }
    }

    /**
     * Returns declared motors that require hardware follower activation.
     *
     * @return follower motor declarations
     */
    public Set<MotorDevice> followerMotors() {
        return motorDevices().stream()
                .filter(motor -> motor.follower() != null)
                .collect(java.util.stream.Collectors.toUnmodifiableSet());
    }

    private void activateLease(Object key, Action action, boolean restart) {
        forgetCompletion(key);
        List<LeaseRegistration> existing = leaseTargets.get(key);
        if (existing != null && !restart) {
            return;
        }
        if (existing != null) {
            releaseLease(key);
        }
        CompiledAction compiled = compiledAction(Objects.requireNonNull(action, "action"));
        preemptExclusiveLeases(key, compiled.resources());
        List<LeaseRegistration> registrations = new ArrayList<>(compiled.partitions().size());
        long recency = ++requestSequence;
        leaseReservations.put(key, new LeaseReservation(recency, compiled.resources(), compiled.exclusive()));
        for (Map.Entry<Mechanism, CompiledPartition> entry : compiled.partitions().entrySet()) {
            CompiledPartition partition = entry.getValue();
            MechanismRuntime runtime = runtimeFor(entry.getKey());
            Object runtimeKey = new Object();
            runtime.activateLease(
                    runtimeKey,
                    partition.action(),
                    recency,
                    partition.resources(),
                    actionRootPath(action) + " partition[" + graph.node(entry.getKey()).name() + "]",
                    key,
                    !partition.opaque());
            registrations.add(new LeaseRegistration(runtimeKey, runtime));
        }
        leaseTargets.put(key, List.copyOf(registrations));
        activeLeaseKeys.add(key);
    }

    /**
     * Ends running lifecycle Actions whose resources overlap a newer request. Lifecycle Actions follow WPILib
     * command semantics, where a conflicting command ends rather than continuing without its resource, so
     * they are cancelled (onEnd(true)) instead of being starved like motor-output leases.
     */
    private void preemptExclusiveLeases(Object key, Set<Object> resources) {
        if (resources.isEmpty()) return;
        for (Object running : List.copyOf(activeLeaseKeys)) {
            if (running == key) continue;
            LeaseReservation reservation = leaseReservations.get(running);
            if (reservation == null || !reservation.exclusive()) continue;
            if (!overlaps(reservation.resources(), resources) || !leaseRunning(running)) continue;
            cancelSpawned(running, Collections.newSetFromMap(new IdentityHashMap<>()));
        }
    }

    private boolean leaseRunning(Object key) {
        List<LeaseRegistration> registrations = leaseTargets.get(key);
        if (registrations == null) return false;
        for (LeaseRegistration registration : registrations) {
            if (!registration.runtime().leaseComplete(registration.runtimeKey())) return true;
        }
        return false;
    }

    private void releaseLease(Object key) {
        List<LeaseRegistration> registrations = leaseTargets.remove(key);
        leaseReservations.remove(key);
        leaseInterrupts.remove(key);
        if (registrations != null) {
            removeIdentity(activeLeaseKeys, key);
            for (LeaseRegistration registration : registrations) {
                registration.runtime().releaseLease(registration.runtimeKey());
            }
        }
    }

    private CompiledAction compiledAction(Action action) {
        CompiledAction cached = compiledActions.get(action);
        if (cached != null) {
            return cached;
        }
        CompiledAction compiled = compileAction(action);
        compiledActions.put(action, compiled);
        return compiled;
    }

    private CompiledAction compileAction(Action action) {
        Actions.validate(action);
        validateConcurrentResourceConflicts(
                action, actionRootPath(action), Collections.newSetFromMap(new IdentityHashMap<>()));
        Map<Mechanism, List<Action>> partitions = new IdentityHashMap<>();
        RequestTarget direct = ambiguousActionTargets.contains(action) ? null : actionTargets.get(action);
        if (direct != null) {
            partitions.computeIfAbsent(direct.root(), ignored -> new ArrayList<>()).add(action);
        } else if (action instanceof Actions.Parallel parallel) {
            for (Action child : parallel.Actions()) {
                addPartition(partitions, child);
            }
        } else {
            addPartition(partitions, action);
        }

        Map<Mechanism, CompiledPartition> compiledPartitions = new IdentityHashMap<>();
        Set<Object> allResources = new LinkedHashSet<>();
        for (Map.Entry<Mechanism, List<Action>> entry : partitions.entrySet()) {
            Action partitioned = entry.getValue().size() == 1
                    ? entry.getValue().get(0)
                    : Actions.parallel(entry.getValue().toArray(Action[]::new));
            Set<Object> resources = actionResources(partitioned);
            if (containsOpaqueRuntimeGraph(partitioned)) {
                RequestTarget target = targetFor(partitioned);
                if (target != null) {
                    Set<Object> conservativeResources = new LinkedHashSet<>(resources);
                    conservativeResources.addAll(resourcesFor(target.owner()));
                    resources = Set.copyOf(conservativeResources);
                }
            }
            CompiledPartition compiled = new CompiledPartition(
                    partitioned, resources, containsOpaqueRuntimeGraph(partitioned));
            compiledPartitions.put(entry.getKey(), compiled);
            allResources.addAll(resources);
        }
        return new CompiledAction(compiledPartitions, allResources, LifecycleAction.present(action));
    }

    private void validateConcurrentResourceConflicts(Action action, String path, Set<Action> visited) {
        if (action == null || !visited.add(action)) return;
        if (action instanceof Actions.Parallel parallel) {
            validateConcurrentGroup(path + ".parallel", parallel.Actions());
            validateConcurrentChildren(parallel.Actions(), path + ".parallel", visited);
        } else if (action instanceof Actions.Race race) {
            validateConcurrentGroup(path + ".race", race.Actions());
            validateConcurrentChildren(race.Actions(), path + ".race", visited);
        } else if (action instanceof Actions.Deadline deadline) {
            validateConcurrentGroup(path + ".deadline", deadline.Actions());
            validateConcurrentChildren(deadline.Actions(), path + ".deadline", visited);
        } else if (action instanceof Actions.Sequence sequence) {
            for (int index = 0; index < sequence.steps().size(); index++) {
                validateConcurrentResourceConflicts(
                        sequence.steps().get(index).action(), path + ".sequence.step[" + index + "]", visited);
            }
            validateConcurrentResourceConflicts(sequence.next(), path + ".sequence.next", visited);
        } else if (action instanceof Actions.Cycle cycle) {
            for (int index = 0; index < cycle.steps().size(); index++) {
                validateConcurrentResourceConflicts(
                        cycle.steps().get(index).action(), path + ".cycle.step[" + index + "]", visited);
            }
        } else if (action instanceof Actions.Choice choice) {
            validateConcurrentResourceConflicts(choice.active(), path + ".choice.active", visited);
            validateConcurrentResourceConflicts(choice.inactive(), path + ".choice.inactive", visited);
        } else if (action instanceof Actions.WhenBranch branch) {
            validateConcurrentResourceConflicts(branch.active(), path + ".when.active", visited);
        } else if (action instanceof Actions.Either either) {
            validateConcurrentResourceConflicts(either.active(), path + ".either.active", visited);
            validateConcurrentResourceConflicts(either.inactive(), path + ".either.inactive", visited);
        } else if (action instanceof Actions.Timeout timeout) {
            validateConcurrentResourceConflicts(timeout.action(), path + ".timeout", visited);
        } else if (action instanceof Actions.WithinTolerance within) {
            validateConcurrentResourceConflicts(within.action(), path + ".withinTolerance", visited);
        } else if (action instanceof Actions.VelocityContribution contribution) {
            validateConcurrentResourceConflicts(contribution.driveAction(), path + ".velocityDrive", visited);
        } else if (action instanceof Actions.Conditional conditional) {
            validateConcurrentResourceConflicts(conditional.action(), path + ".conditional.action", visited);
            validateConcurrentResourceConflicts(conditional.next(), path + ".conditional.next", visited);
        } else if (action instanceof Actions.Then then) {
            validateConcurrentResourceConflicts(then.action(), path + ".then.action", visited);
            validateConcurrentResourceConflicts(then.next(), path + ".then.next", visited);
        } else if (action instanceof PathAction pathAction) {
            // Markers may overlap each other or the path drive; the later-triggered output wins at
            // runtime. Each marker's own concurrent groups must still be conflict-free.
            pathAction.markers().forEach((name, marker) ->
                    validateConcurrentResourceConflicts(marker, path + ".marker[" + name + "]", visited));
        }
    }

    private void validateConcurrentChildren(List<Action> children, String path, Set<Action> visited) {
        for (int index = 0; index < children.size(); index++) {
            validateConcurrentResourceConflicts(children.get(index), path + ".child[" + index + "]", visited);
        }
    }

    private void validateConcurrentGroup(String path, List<Action> children) {
        Map<Object, ConcurrentClaim> claimed = new LinkedHashMap<>();
        for (int index = 0; index < children.size(); index++) {
            Action child = children.get(index);
            Set<Object> resources = actionResources(child);
            for (Object resource : resources) {
                ConcurrentClaim existing = claimed.get(resource);
                if (existing != null) {
                    throw new IllegalArgumentException(
                            "Concurrent action conflict at " + path + ": child[" + existing.index() + "] "
                                    + actionDescription(existing.action()) + " and child[" + index + "] "
                                    + actionDescription(child) + " both claim " + resourceDescription(resource)
                                    + ". Put them in a sequence or remove one of the duplicate outputs.");
                }
                claimed.put(resource, new ConcurrentClaim(index, child));
            }
        }
    }

    private String actionDescription(Action action) {
        String type = action.getClass().getSimpleName();
        String safeType = type.isBlank() ? action.getClass().getName() : type;
        List<String> locations = actionDeclarationLocations.get(action);
        return locations == null || locations.isEmpty()
                ? safeType + " (inline/undeclared)"
                : safeType + " declared as " + String.join(" or ", locations);
    }

    private String actionRootPath(Action action) {
        List<String> locations = actionDeclarationLocations.get(action);
        if (locations != null && !locations.isEmpty()) return String.join("|", locations);
        String type = action.getClass().getSimpleName();
        return "requestedAction<" + (type.isBlank() ? action.getClass().getName() : type) + ">";
    }

    private static String resourceDescription(Object resource) {
        if (resource instanceof MotorDevice motor) return "motor '" + motor.defaultName() + "'";
        return "resource '" + resource + "'";
    }

    private record ConcurrentClaim(int index, Action action) {
    }

    private boolean containsOpaqueRuntimeGraph(Action action) {
        if (action == null) return false;
        if (action instanceof PathAction path) {
            PathRuntime runtime = pathRuntimes.get(path);
            return runtime == null || runtime.ownership(path).isEmpty();
        }
        if (action instanceof Actions.Computed) return false;
        for (Action.Child child : action.children()) {
            if (containsOpaqueRuntimeGraph(child.action())) return true;
        }
        return false;
    }

    private void addPartition(Map<Mechanism, List<Action>> partitions, Action action) {
        RequestTarget target = targetFor(action);
        if (target == null) {
            throw new IllegalArgumentException("Action is not owned by a registered mechanism and does not target "
                    + "a declaration owned by one.");
        }
        partitions.computeIfAbsent(target.root(), ignored -> new ArrayList<>()).add(action);
    }

    private Set<Object> actionResources(Action action) {
        Set<Object> resources = new LinkedHashSet<>();
        for (Object declaration : actionDeclarations(action)) {
            collectOwnedResource(declaration, resources);
        }
        collectPathResources(action, resources);
        return Set.copyOf(resources);
    }

    private Set<Object> resourcesFor(Mechanism owner) {
        Set<Object> resources = new LinkedHashSet<>();
        for (Object declaration : graph.declarations(List.of(owner))) {
            collectOwnedResource(declaration, resources);
        }
        return Set.copyOf(resources);
    }

    private void collectOwnedResource(Object declaration, Set<Object> resources) {
        if (declaration instanceof ControlBinding<?> control && control.sink() != null) {
            resources.add(control.sink());
        }
        if (declaration instanceof MotorDevice motor) {
            resources.add(motor);
        } else if (declaration instanceof NamedResource named) {
            resources.add(named);
        } else if (declaration instanceof ControlBinding<?> control) {
            resources.addAll(control.motors());
        } else if (declaration instanceof Mechanism mechanism) {
            resources.addAll(resourcesFor(mechanism));
        } else if (declaration instanceof Action action) {
            for (Object ownedDeclaration : actionDeclarations(action)) {
                if (ownedDeclaration != action) collectOwnedResource(ownedDeclaration, resources);
            }
            collectPathResources(action, resources);
        } else if (declaration instanceof Iterable<?> values) {
            values.forEach(value -> collectOwnedResource(value, resources));
        }
    }

    private void collectPathResources(Action action, Set<Object> resources) {
        if (action == null) return;
        if (action instanceof PathAction path) {
            PathRuntime runtime = pathRuntimes.get(path);
            if (runtime != null) runtime.ownership(path).forEach(value -> collectOwnedResource(value, resources));
        }
        for (Action.Child child : action.children()) {
            collectPathResources(child.action(), resources);
        }
    }

    /**
     * Requests an action, restarting an existing request and making it the newest lease.
     * Newer leases win conflicting resources; older leases remain active and may resume
     * after the newer lease releases.
     *
     * @param action action owned by, or targeting a declaration owned by, a registered mechanism
     * @return this scheduler
     * @throws IllegalArgumentException when ownership cannot be inferred
     */
    public MechanismScheduler request(Action action) {
        Action requested = Objects.requireNonNull(action, "action");
        if (requested instanceof Actions.Abort abort) {
            pendingAborts.add(abort.mechanism());
            return this;
        }
        Action safeAction = rootAction(requested);
        for (int index = 0; index < evaluatingLeases.size(); index++) {
            if (evaluatingLeases.get(index) == safeAction) {
                // An action restarting itself (or an ancestor lease) from inside its own tree, for
                // example sequence().doOnce(...).then(doOnce(() -> self.request())): restart it after
                // this cycle instead of tearing down the tree that is evaluating right now.
                deferredRequests.add(safeAction);
                return this;
            }
        }
        Interrupt interrupt = requested instanceof Actions.Interruptible policy ? policy.interrupt() : null;
        if (blockedBy(safeAction)) {
            // A running Action does not allow interruption yet: hold this request instead of applying it.
            forgetCompletion(safeAction);
            pendingRequests.removeIf(pending -> pending.action() == safeAction);
            pendingRequests.add(new PendingRequest(safeAction, interrupt));
        } else {
            pendingRequests.removeIf(pending -> pending.action() == safeAction);
            activateLease(safeAction, safeAction, true);
            setInterrupt(safeAction, interrupt);
        }
        // A request made while a lease evaluates (for example doOnce(() -> intake.request()))
        // belongs to that lease and is cancelled with it. Any other request is independent, even
        // when the same action was previously spawned by a lease.
        for (Set<Action> spawned : spawnedLeases.values()) spawned.remove(safeAction);
        if (!evaluatingLeases.isEmpty()) {
            Object parent = evaluatingLeases.get(evaluatingLeases.size() - 1);
            if (parent != null && parent != safeAction) {
                spawnedLeases.computeIfAbsent(parent, ignored -> Collections.newSetFromMap(new IdentityHashMap<>()))
                        .add(safeAction);
            }
        }
        return this;
    }

    /**
     * Returns whether a running lease with a restrictive interrupt policy overlaps the action's
     * resources and does not allow interruption yet.
     */
    private boolean blockedBy(Action action) {
        Set<Object> resources = compiledAction(action).resources();
        for (int index = 0; index < activeLeaseKeys.size(); index++) {
            Object key = activeLeaseKeys.get(index);
            Interrupt running = leaseInterrupts.get(key);
            if (running == null || !running.restrictive()) continue;
            LeaseReservation reservation = leaseReservations.get(key);
            if (reservation == null || !overlaps(reservation.resources(), resources)) continue;
            if (!running.allowsInterrupt(false)) return true;
        }
        return false;
    }

    private static boolean overlaps(Set<Object> running, Set<Object> requested) {
        for (Object resource : requested) {
            if (running.contains(resource)) return true;
        }
        return false;
    }

    private void setInterrupt(Action action, Interrupt interrupt) {
        if (interrupt == null) {
            leaseInterrupts.remove(action);
        } else {
            leaseInterrupts.put(action, interrupt);
        }
    }

    private static Action rootAction(Action action) {
        return action instanceof Actions.Interruptible policy ? policy.action() : action;
    }

    /**
     * Starts held requests whose blockers have completed or allow interruption now, in request order.
     * While the robot is disabled, interrupt policies are overridden: held requests start (and resume
     * when enabled) instead of waiting for the running Action.
     */
    private void activatePending(boolean enabled) {
        for (Mechanism mechanism : List.copyOf(pendingAborts)) {
            abortMechanism(mechanism);
        }
        pendingAborts.clear();
        for (int index = 0; index < pendingRequests.size(); ) {
            PendingRequest pending = pendingRequests.get(index);
            if (enabled && blockedBy(pending.action())) {
                index++;
                continue;
            }
            pendingRequests.remove(index);
            activateLease(pending.action(), pending.action(), true);
            setInterrupt(pending.action(), pending.interrupt());
        }
    }

    /**
     * Cancels every running lease that touches the mechanism, ignoring interrupt policies. Held newer
     * requests are not cancelled: they proceed in the same cycle, as they would after the running Action.
     */
    private void abortMechanism(Mechanism mechanism) {
        MechanismRuntime runtime = runtimes.get(mechanism);
        if (runtime != null) {
            for (Object key : List.copyOf(activeLeaseKeys)) {
                List<LeaseRegistration> registrations = leaseTargets.get(key);
                if (registrations == null) continue;
                for (LeaseRegistration registration : registrations) {
                    if (registration.runtime() == runtime) {
                        cancelSpawned(key, Collections.newSetFromMap(new IdentityHashMap<>()));
                        break;
                    }
                }
            }
        }
    }

    private record PendingRequest(Action action, Interrupt interrupt) {
    }

    /**
     * Aborts every committed Action on a mechanism. Requests made while a lease evaluates are applied at the
     * start of the next cycle. Interrupt policies do not protect an Action from abort; disable does too.
     */
    public MechanismScheduler abort(Mechanism mechanism) {
        pendingAborts.add(Objects.requireNonNull(mechanism, "mechanism"));
        return this;
    }

    /**
     * Cancels an action and every lease it requested while it was running. Leases for actions
     * that merely appear inside its tree but were requested independently (for example by a
     * button) keep running. Resources no longer driven by another lease are neutralized during
     * the next output cycle.
     */
    public MechanismScheduler cancel(Action action) {
        cancelSpawned(rootAction(Objects.requireNonNull(action, "action")),
                Collections.newSetFromMap(new IdentityHashMap<>()));
        return this;
    }

    /**
     * Cancels every running lease and held request, ending lifecycle Actions with onEnd(true). Used when
     * the owning robot runtime closes.
     *
     * @return this scheduler
     */
    public MechanismScheduler cancelAll() {
        pendingRequests.clear();
        for (Object key : List.copyOf(activeLeaseKeys)) {
            cancelSpawned(key, Collections.newSetFromMap(new IdentityHashMap<>()));
        }
        return this;
    }

    /** Returns whether a requested action has an active, incomplete lease. */
    public boolean isRunning(Action action) {
        List<LeaseRegistration> registrations = leaseTargets.get(rootAction(Objects.requireNonNull(action, "action")));
        if (registrations == null) return false;
        for (LeaseRegistration registration : registrations) {
            if (!registration.runtime().leaseComplete(registration.runtimeKey())) return true;
        }
        return false;
    }

    /**
     * Returns whether a requested action reached completion. Cancellation clears completion,
     * and requesting the same action again restarts it and clears the previous result.
     */
    public boolean isComplete(Action action) {
        Action safeAction = rootAction(Objects.requireNonNull(action, "action"));
        List<LeaseRegistration> registrations = leaseTargets.get(safeAction);
        if (registrations == null || registrations.isEmpty()) return completed(safeAction);
        for (LeaseRegistration registration : registrations) {
            if (!registration.runtime().leaseComplete(registration.runtimeKey())) return false;
        }
        return true;
    }

    private void cancelSpawned(Object key, Set<Object> visited) {
        if (key == null || !visited.add(key)) return;
        deferredRequests.removeIf(deferred -> deferred == key);
        forgetCompletion(key);
        pendingRequests.removeIf(pending -> pending.action() == key);
        releaseLease(key);
        Set<Action> spawned = spawnedLeases.remove(key);
        if (spawned == null) return;
        for (Action child : List.copyOf(spawned)) cancelSpawned(child, visited);
    }

    /**
     * Contains runtime exceptions per mechanism and lease instead of propagating them out of
     * {@link #periodic}. A faulted lease contributes no outputs that cycle (its motors go neutral)
     * and the fault is queued for {@link #drainRuntimeFaults()}. Exceptions raised under a
     * {@code PANIC} failure policy still propagate. Disabled by default so tests and tools fail fast.
     */
    public MechanismScheduler isolateFaults(boolean isolate) {
        isolateFaults = isolate;
        return this;
    }

    public boolean isolatesFaults() {
        return isolateFaults;
    }

    /**
     * Returns and clears faults to report. Each distinct fault is returned at most once every
     * five seconds while it keeps occurring; {@link RuntimeFault#occurrences()} counts repeats.
     */
    public List<RuntimeFault> drainRuntimeFaults() {
        for (String message : FaultIsolation.drainReports()) {
            pendingFaults.add(new RuntimeFault("Actions.fault", new IllegalStateException(message), 1));
            RuntimeEvents.emit(RuntimeEvents.FAULT, message);
        }
        if (pendingFaults.isEmpty()) return List.of();
        List<RuntimeFault> drained = List.copyOf(pendingFaults);
        pendingFaults.clear();
        return drained;
    }

    /**
     * Returns and clears rate-limited warnings for runtime values that were held or neutralized
     * instead of throwing (for example a non-finite dynamic target or interpolation input).
     */
    public List<String> drainRuntimeWarnings() {
        return FaultIsolation.drainWarnings();
    }

    /** Returns the tuning lock conditions that gate dashboard writes. */
    public TuningLocks tuningLocks() {
        return resolver.overrides().locks();
    }

    /**
     * Replaces the tuning lock conditions. Call before the telemetry schema is read; a schema already
     * built is rebuilt on the next read.
     *
     * @param locks lock conditions
     * @return this scheduler
     */
    public MechanismScheduler tuningLocks(TuningLocks locks) {
        resolver.overrides().locks(Objects.requireNonNull(locks, "locks"));
        telemetrySchema = null;
        return this;
    }

    /**
     * Attaches the persistent store used by persisted telemetry values and by interpolation points marked
     * persisted. Attach before the telemetry schema is read; a schema already built is rebuilt on the next
     * read.
     *
     * @param store store, or null to disable persistence
     * @param matchTimeSeconds supplier of the FMS match time reported with writes (may be null)
     * @return this scheduler
     */
    /** Returns the persistence context that binds persisted telemetry values, or the disabled context. */
    public TelemetryPersistence telemetryPersistence() {
        return resolver.overrides().persistence();
    }

    public MechanismScheduler persistentStore(PersistentStore store, DoubleSupplier matchTimeSeconds) {
        TelemetryPersistence persistence = new TelemetryPersistence(store, matchTimeSeconds);
        persistence.keep(Set.of(PowerRuntime.BATTERY_OHMS_KEY));
        resolver.overrides().persistence(persistence);
        power.batteryStore(store);
        telemetrySchema = null;
        return this;
    }

    /** Returns the characterizations that persist through the store, in registration order. */
    public List<Characterization> characterizations() {
        return List.copyOf(characterizations);
    }

    /**
     * Forwards a disable to every persisted characterization (history snapshot when enabled more than 30 s).
     *
     * @param enabledSeconds how long the robot was enabled in the period that ended
     */
    public void recordDisable(double enabledSeconds) {
        for (Characterization characterization : characterizations) {
            characterization.recordDisable(enabledSeconds);
        }
    }

    /** Forwards a match end to every persisted characterization. */
    public void recordMatchEnd() {
        for (Characterization characterization : characterizations) {
            characterization.recordMatchEnd();
        }
    }

    /**
     * Attaches the persistent store to the characterizations declared by a mechanism tree: each is keyed by its
     * mechanism path plus declaration name, and its persisted model is restored before the robot runs.
     */
    private void attachCharacterizations(Mechanism mechanism) {
        TelemetryPersistence persistence = resolver.overrides().persistence();
        if (!persistence.enabled()) {
            return;
        }
        MechanismNode node = graph.node(mechanism);
        collectCharacterizations(node, node.name(), persistence, new IdentityHashMap<>());
    }

    private void collectCharacterizations(
            MechanismNode node,
            String path,
            TelemetryPersistence persistence,
            IdentityHashMap<Object, Boolean> visited) {
        if (visited.put(node, Boolean.TRUE) != null) {
            return;
        }
        node.declarations().forEach((name, declaration) -> {
            if (!(declaration instanceof Characterization characterization)) {
                return;
            }
            if (containsIdentity(characterizations, characterization)) {
                return;
            }
            String key = path + "/" + name;
            persistence.keep(PersistentModelStorage.keys(key));
            characterization.persistAs(new PersistentModelStorage(persistence.store()), key);
            characterizations.add(characterization);
        });
        node.children().forEach((childName, child) ->
                collectCharacterizations(graph.node(child), path + "/" + childName, persistence, visited));
    }

    /**
     * Returns a one-line explanation of what the mechanism is doing with its outputs this cycle: running an
     * action, starved by a newer lease, superseded, faulted, disabled, or idle. Published as {@code State/Reason}.
     */
    public String reason(Mechanism mechanism) {
        MechanismRuntime runtime = runtimes.get(mechanism);
        if (runtime == null) {
            RequestTarget target = declarationTargets.get(mechanism);
            runtime = target == null ? null : runtimes.get(target.root());
        }
        return runtime == null ? "Idle" : runtime.reason();
    }

    /** Returns the fault its root mechanism raised this cycle (telemetry), or an empty string when healthy. */
    public String fault(Mechanism mechanism) {
        MechanismRuntime runtime = runtimes.get(mechanism);
        if (runtime == null) {
            RequestTarget target = declarationTargets.get(mechanism);
            runtime = target == null ? null : runtimes.get(target.root());
        }
        return runtime == null || !runtime.faultedThisCycle() ? "" : runtime.lastFault();
    }

    private void isolate(
            MechanismRuntime runtime,
            String phase,
            RuntimeException exception,
            Set<MechanismRuntime> faulted) {
        if (!isolateFaults || FaultIsolation.isPanic(exception)) throw exception;
        faulted.add(runtime);
        runtime.markFault(phase, exception);
        recordFault(runtime.name() + " " + phase, exception);
    }

    private void recordFault(String location, RuntimeException exception) {
        if (!isolateFaults) throw exception;
        String key = location + "|" + exception.getClass().getName();
        FaultHistory history = faultHistory.get(key);
        if (history == null) {
            if (faultHistory.size() >= COMPLETION_HISTORY_LIMIT) {
                faultHistory.remove(faultHistory.keySet().iterator().next());
            }
            history = new FaultHistory();
            faultHistory.put(key, history);
        }
        history.occurrences++;
        if (history.reported && cycleSeconds - history.lastReportSeconds < FAULT_REPORT_PERIOD_SECONDS
                && cycleSeconds >= history.lastReportSeconds) {
            return;
        }
        history.reported = true;
        history.lastReportSeconds = cycleSeconds;
        pendingFaults.add(new RuntimeFault(location, exception, history.occurrences));
        RuntimeEvents.emit(RuntimeEvents.FAULT, location + ": " + exception.getClass().getSimpleName()
                + " " + exception.getMessage());
        if (pendingFaults.size() > COMPLETION_HISTORY_LIMIT) pendingFaults.remove(0);
    }

    /** A contained runtime fault: where it happened, the cause, and how often it has occurred. */
    public record RuntimeFault(String location, RuntimeException exception, int occurrences) {
        public RuntimeFault {
            Objects.requireNonNull(location, "location");
            Objects.requireNonNull(exception, "exception");
        }

        public String message() {
            String detail = exception.getMessage();
            return "Athena mechanism fault in " + location + " (outputs neutralized; occurrences="
                    + occurrences + "): " + exception.getClass().getSimpleName()
                    + (detail == null ? "" : ": " + detail);
        }
    }

    private static final class FaultHistory {
        private int occurrences;
        private boolean reported;
        private double lastReportSeconds;
    }

    public Action action(Mechanism mechanism) {
        return runtimeFor(mechanism).action();
    }

    public List<ResolvedOutput> periodic(MechanismContext mechanismContext, EventContext eventContext) {
        return ca.frc6390.athena.hardware.runtime.RuntimeHardwareAccess.call(
                actionContext,
                () -> periodicBound(mechanismContext, eventContext));
    }

    private List<ResolvedOutput> periodicBound(MechanismContext mechanismContext, EventContext eventContext) {
        MechanismContext safeMechanismContext = mechanismContext == null ? MechanismContext.empty() : mechanismContext;
        activatePending(safeMechanismContext.enabled());
        sampleSignals();
        List<ResolvedOutput> outputs = new ArrayList<>(outputCapacityHint);
        Map<MechanismRuntime, Set<MotorDevice>> drivenByRuntime = drivenByRuntimeScratch;
        Set<MotorDevice> globallyDriven = globallyDrivenScratch;
        drivenByRuntime.clear();
        globallyDriven.clear();
        cycleSeconds = safeMechanismContext.nowSeconds();
        Set<MechanismRuntime> faulted = Collections.newSetFromMap(new IdentityHashMap<>());
        try {
            for (MechanismRuntime runtime : runtimes.values()) {
                try {
                    runtime.runHooks(eventContext, false);
                } catch (RuntimeException exception) {
                    isolate(runtime, "hooks", exception, faulted);
                }
            }
            evaluateStagings(safeMechanismContext.nowSeconds());
            // Evaluate every lease first so reservations reflect where each lease is in its tree
            // this cycle, then arbitrate and apply against those reservations.
            for (MechanismRuntime runtime : runtimes.values()) {
                try {
                    runtime.evaluateCandidates(safeMechanismContext);
                } catch (RuntimeException exception) {
                    evaluatingLeases.clear();
                    isolate(runtime, "evaluation", exception, faulted);
                }
            }
            Map<Object, Long> reservations = globalLeaseReservations();
            // Electrical and thermal caps use this cycle's controller outputs and are applied before any
            // motor command below.
            power.cycle(powerMotors, actionContext, resolver.overrides(), safeMechanismContext.nowSeconds());
            Map<ControlBinding<?>, StageCaps> caps = stageCapsByControl();
            Map<ControlBinding<?>, Enum<?>> stageNames = stageNamesByControl(caps);
            for (MechanismRuntime runtime : runtimes.values()) {
                runtime.stageCaps(caps, stageNames);
                runtime.electrical(power);
            }
            for (MechanismRuntime runtime : runtimes.values()) {
                Set<MotorDevice> driven;
                if (faulted.contains(runtime)) {
                    driven = runtime.neutralizeAfterFault();
                } else {
                    try {
                        driven = runtime.applyCandidates(outputs, reservations);
                    } catch (RuntimeException exception) {
                        isolate(runtime, "output", exception, faulted);
                        driven = runtime.neutralizeAfterFault();
                    }
                }
                drivenByRuntime.put(runtime, driven);
                globallyDriven.addAll(driven);
            }
        } finally {
            digitalInputs.forEach(DigitalInputDevice::clearLatchedEdges);
            for (MechanismRuntime runtime : runtimes.values()) {
                runtime.finishOutputCycle(globallyDriven, drivenByRuntime.getOrDefault(runtime, Set.of()));
            }
            retireCompletedRequests();
            evaluatingLeases.clear();
            if (!deferredRequests.isEmpty()) {
                List<Action> deferred = List.copyOf(deferredRequests);
                deferredRequests.clear();
                deferred.forEach(this::request);
            }
        }
        outputCapacityHint = Math.max(outputCapacityHint, outputs.size());
        resolver.overrides().persistence().periodic(cycleSeconds,
                eventContext == null ? LifecycleMode.ROBOT : eventContext.mode(),
                safeMechanismContext.enabled());
        return outputs;
    }

    /** Returns the electrical and thermal state from the latest cycle, for telemetry. */
    public PowerSnapshot powerSnapshot() {
        return power.snapshot();
    }

    /** Returns the robot-wide power policy in effect (defaults when none is declared). */
    public PowerPolicy powerPolicy() {
        return power.policy();
    }

    /**
     * Sets the supplier of the measured battery voltage used by the power pass. Typically the robot
     * controller battery voltage. A non-finite reading disables budgeting for that cycle.
     *
     * @param busVoltage battery voltage in volts
     * @return this scheduler
     */
    public MechanismScheduler busVoltage(java.util.function.DoubleSupplier busVoltage) {
        power.busVoltage(Objects.requireNonNull(busVoltage, "busVoltage"));
        return this;
    }

    /**
     * Returns and clears recorded stage transitions across all declared stagings, oldest first.
     *
     * @return transitions since the last call
     */
    public List<Staging.Change<?>> drainStageChanges() {
        List<Staging.Change<?>> changes = new ArrayList<>();
        for (Staging<?> staging : stagings) {
            changes.addAll(staging.drainChanges());
        }
        return changes;
    }

    private void refreshPowerDeclarations() {
        stagings.clear();
        PowerPolicy policy = null;
        for (Object declaration : graph.declarations(runtimes.keySet())) {
            if (declaration instanceof Staging<?> staging && !containsIdentity(stagings, staging)) {
                stagings.add(staging);
            } else if (declaration instanceof PowerPolicy declared) {
                if (policy != null && policy != declared) {
                    throw new IllegalStateException("Only one PowerPolicy may be declared per robot.");
                }
                policy = declared;
            }
        }
        if (policy != null) {
            power.policy(policy);
        }
        powerMotors = List.copyOf(motorDevices());
    }

    private static boolean containsIdentity(List<?> values, Object value) {
        for (Object existing : values) {
            if (existing == value) return true;
        }
        return false;
    }

    private void evaluateStagings(double nowSeconds) {
        for (Staging<?> staging : stagings) {
            try {
                staging.evaluate(nowSeconds, stageHost);
            } catch (RuntimeException exception) {
                if (FaultIsolation.isPanic(exception)) throw exception;
                recordFault("staging " + staging.currentStage(), exception);
            }
        }
    }

    /** Maps each capped binding to the stage constant that set its caps, for the State/Reason explanation. */
    private Map<ControlBinding<?>, Enum<?>> stageNamesByControl(Map<ControlBinding<?>, StageCaps> caps) {
        if (caps.isEmpty()) return Map.of();
        Map<ControlBinding<?>, Enum<?>> names = new IdentityHashMap<>();
        for (Staging<?> staging : stagings) {
            for (StageModifier modifier : staging.activeModifiers()) {
                names.put(modifier.binding(), staging.currentStage());
            }
        }
        return names;
    }

    private Map<ControlBinding<?>, StageCaps> stageCapsByControl() {
        if (stagings.isEmpty()) return Map.of();
        Map<ControlBinding<?>, StageCaps> caps = new IdentityHashMap<>();
        for (Staging<?> staging : stagings) {
            for (StageModifier modifier : staging.activeModifiers()) {
                caps.put(modifier.binding(),
                        caps.getOrDefault(modifier.binding(), StageCaps.NONE).with(modifier.kind(), modifier.fraction()));
            }
        }
        return caps;
    }

    private void retireCompletedRequests() {
        List<Object> completed = retirementScratch;
        completed.clear();
        for (int keyIndex = 0; keyIndex < activeLeaseKeys.size(); keyIndex++) {
            Object key = activeLeaseKeys.get(keyIndex);
            if (!(key instanceof Action)) continue;
            List<LeaseRegistration> registrations = leaseTargets.get(key);
            if (!registrations.isEmpty() && registrationsComplete(registrations)) {
                completed.add(key);
            }
        }
        for (Object key : completed) {
            rememberCompletion(key);
            releaseLease(key);
        }
    }

    private void rememberCompletion(Object key) {
        forgetCompletion(key);
        completedLeaseKeys.add(key);
        if (completedLeaseKeys.size() > COMPLETION_HISTORY_LIMIT) completedLeaseKeys.remove(0);
    }

    private void forgetCompletion(Object key) {
        completedLeaseKeys.removeIf(existing -> existing == key);
    }

    private boolean completed(Object key) {
        for (int index = 0; index < completedLeaseKeys.size(); index++) {
            if (completedLeaseKeys.get(index) == key) return true;
        }
        return false;
    }

    private Map<Object, Long> globalLeaseReservations() {
        Map<Object, Long> reservations = reservationScratch;
        reservations.clear();
        for (int keyIndex = 0; keyIndex < activeLeaseKeys.size(); keyIndex++) {
            Object key = activeLeaseKeys.get(keyIndex);
            List<LeaseRegistration> registrations = leaseTargets.get(key);
            if (registrations == null || registrationsComplete(registrations)) {
                continue;
            }
            LeaseReservation lease = leaseReservations.get(key);
            for (int index = 0; index < registrations.size(); index++) {
                LeaseRegistration registration = registrations.get(index);
                for (Object resource : registration.runtime().leaseReservation(registration.runtimeKey())) {
                    reservations.merge(resource, lease.recency(), Math::max);
                }
            }
        }
        return reservations;
    }

    private static void removeIdentity(List<Object> values, Object target) {
        for (int index = 0; index < values.size(); index++) {
            if (values.get(index) == target) {
                values.remove(index);
                return;
            }
        }
    }

    private static boolean registrationsComplete(List<LeaseRegistration> registrations) {
        for (int index = 0; index < registrations.size(); index++) {
            LeaseRegistration registration = registrations.get(index);
            if (!registration.runtime().leaseComplete(registration.runtimeKey())) return false;
        }
        return true;
    }

    private void refreshDigitalInputs() {
        digitalInputs.clear();
        for (Object declaration : graph.declarations(runtimes.keySet())) {
            if (declaration instanceof DigitalInputDevice digitalInput) {
                digitalInputs.add(digitalInput);
            }
        }
    }

    public List<ResolvedOutput> robotPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.ROBOT, LifecyclePhase.PERIODIC, true, false));
    }

    public List<ResolvedOutput> robotInit(double nowSeconds, double dtSeconds) {
        return periodic(new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.ROBOT, LifecyclePhase.INIT, true, false));
    }

    public List<ResolvedOutput> teleopPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.TELEOP, LifecyclePhase.PERIODIC, true, false));
    }

    public List<ResolvedOutput> autoPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(new MechanismContext(nowSeconds, 0.0, dtSeconds, true, true, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.AUTONOMOUS, LifecyclePhase.PERIODIC, true, false));
    }

    public List<ResolvedOutput> disabledPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(new MechanismContext(nowSeconds, 0.0, dtSeconds, false, false, false),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.DISABLED, LifecyclePhase.PERIODIC, false, false));
    }

    public List<ResolvedOutput> simulationPeriodic(double nowSeconds, double dtSeconds) {
        return periodic(new MechanismContext(nowSeconds, 0.0, dtSeconds, true, false, true),
                new EventContext(nowSeconds, dtSeconds, LifecycleMode.SIMULATION, LifecyclePhase.PERIODIC, true, true));
    }

    private MechanismRuntime runtime(Mechanism mechanism) {
        MechanismNode node = graph.node(mechanism);
        Map<String, Object> runtimeDeclarations = new LinkedHashMap<>(node.declarations());
        int declarationIndex = 0;
        for (Object declaration : graph.declarations(List.of(mechanism))) {
            if (!runtimeDeclarations.containsValue(declaration)) {
                runtimeDeclarations.put("runtimeDeclaration" + declarationIndex++, declaration);
            }
        }
        MechanismNode runtimeNode = new MechanismNode(
                node.name(),
                node.mechanism(),
                node.children(),
                node.Actions(),
                runtimeDeclarations,
                graph.hooks(mechanism));
        HookRuntime.LeaseController leases = new HookRuntime.LeaseController() {
            @Override
            public void activate(Object key, Action action, boolean restart) {
                activateLease(key, action, restart);
            }

            @Override
            public void release(Object key) {
                releaseLease(key);
            }
        };
        return MechanismRuntime.of(runtimeNode, actionContext, resolver, pathRuntimes, leases)
                .integration(leaseResources, runtimeHooks)
                .simulationStep(simulationStep);
    }

    private MechanismRuntime runtimeFor(Mechanism mechanism) {
        MechanismRuntime runtime = runtimes.get(mechanism);
        if (runtime == null) {
            throw new IllegalArgumentException("Mechanism is not registered: " + mechanism.getClass().getName());
        }
        return runtime;
    }

    private void indexActionTargets(
            Mechanism root,
            Mechanism mechanism,
            String mechanismPath,
            Set<Mechanism> visited) {
        if (!visited.add(mechanism)) {
            return;
        }
        MechanismNode node = graph.node(mechanism);
        for (Map.Entry<String, Action> entry : node.Actions().entrySet()) {
            Action action = entry.getValue();
            String declarationLocation = mechanismPath + ".actions[\"" + entry.getKey() + "\"]";
            List<String> locations = actionDeclarationLocations.computeIfAbsent(action, ignored -> new ArrayList<>());
            if (!locations.contains(declarationLocation)) locations.add(declarationLocation);
            if (ambiguousActionTargets.contains(action)) {
                continue;
            }
            RequestTarget existing = actionTargets.get(action);
            RequestTarget target = new RequestTarget(root, mechanism);
            if (existing != null && !existing.equals(target)) {
                actionTargets.remove(action);
                ambiguousActionTargets.add(action);
                continue;
            }
            actionTargets.put(action, target);
        }
        for (Map.Entry<String, Mechanism> entry : node.children().entrySet()) {
            indexActionTargets(root, entry.getValue(), mechanismPath + "." + entry.getKey(), visited);
        }
    }

    private void rebuildCompiledActions() {
        compiledActions.clear();
        Set<Mechanism> visited = Collections.newSetFromMap(new IdentityHashMap<>());
        for (Mechanism root : runtimes.keySet()) {
            compileDeclaredActions(root, visited);
        }
    }

    private void compileDeclaredActions(Mechanism mechanism, Set<Mechanism> visited) {
        if (!visited.add(mechanism)) {
            return;
        }
        MechanismNode node = graph.node(mechanism);
        for (Action action : node.Actions().values()) {
            if (targetFor(action) != null) {
                compiledActions.put(action, compileAction(action));
            }
        }
        for (Mechanism child : node.children().values()) {
            compileDeclaredActions(child, visited);
        }
    }

    private void indexDeclarationTargets(Mechanism root, Mechanism mechanism, Set<Mechanism> visited) {
        if (!visited.add(mechanism)) {
            return;
        }
        MechanismNode node = graph.node(mechanism);
        RequestTarget target = new RequestTarget(root, mechanism);
        indexDeclarationTarget(mechanism, target);
        for (Object declaration : node.declarations().values()) {
            indexDeclarationTarget(declaration, target);
        }
        for (Mechanism child : node.children().values()) {
            indexDeclarationTargets(root, child, visited);
        }
    }

    private void indexDeclarationTarget(Object declaration, RequestTarget target) {
        if (declaration == null) {
            return;
        }
        if (ambiguousDeclarationTargets.contains(declaration)) {
            return;
        }
        RequestTarget existing = declarationTargets.get(declaration);
        if (existing != null && !existing.equals(target)) {
            declarationTargets.remove(declaration);
            ambiguousDeclarationTargets.add(declaration);
            return;
        }
        declarationTargets.put(declaration, target);
        if (declaration instanceof ControlBinding<?> control) {
            for (Object dependency : controlDeclarations(control)) {
                indexDeclarationTarget(dependency, target);
            }
        }
    }

    private RequestTarget targetFor(Action action) {
        RequestTarget direct = ambiguousActionTargets.contains(action) ? null : actionTargets.get(action);
        if (direct != null) {
            return direct;
        }
        Set<Object> declarations = actionDeclarations(action);
        Set<RequestTarget> targets = new HashSet<>();
        for (Object declaration : declarations) {
            RequestTarget target = declarationTargets.get(declaration);
            if (target != null) {
                targets.add(target);
            }
        }
        if (targets.size() > 1) {
            Mechanism sharedRoot = null;
            for (RequestTarget target : targets) {
                if (sharedRoot == null) {
                    sharedRoot = target.root();
                } else if (sharedRoot != target.root()) {
                    throw new IllegalArgumentException(
                            "Action targets declarations owned by multiple registered mechanism roots.");
                }
            }
            return new RequestTarget(sharedRoot, sharedRoot);
        }
        if (!targets.isEmpty()) return targets.iterator().next();
        if (!hasOwnedDeclaration(declarations) && runtimes.size() == 1) {
            Mechanism root = runtimes.keySet().iterator().next();
            return new RequestTarget(root, root);
        }
        return null;
    }

    /** Named resources arbitrate leases but do not select a mechanism root. */
    private static boolean hasOwnedDeclaration(Set<Object> declarations) {
        for (Object declaration : declarations) {
            if (!(declaration instanceof NamedResource)) return true;
        }
        return false;
    }

    private static Set<Object> actionDeclarations(Action action) {
        Set<Object> declarations = new LinkedHashSet<>();
        collectActionDeclarations(action, declarations);
        return declarations;
    }

    private static void collectActionDeclarations(Action action, Set<Object> declarations) {
        if (action == null) {
            return;
        }
        if (action instanceof LifecycleAction lifecycle) {
            declarations.addAll(lifecycle.requirements());
            return;
        }
        if (action instanceof Actions.Computed computed) {
            for (Object owner : computed.ownership()) {
                if (owner instanceof Action ownedAction) collectActionDeclarations(ownedAction, declarations);
                else if (owner instanceof Iterable<?> values) {
                    values.forEach(value -> {
                        if (value instanceof Action ownedAction) collectActionDeclarations(ownedAction, declarations);
                        else declarations.add(value);
                    });
                } else declarations.add(owner);
            }
            return;
        }
        if (action instanceof Actions.HardwareComputed computed) {
            declarations.addAll(computed.declarations());
            return;
        }
        if (action instanceof Actions.ImuYawMutation setYaw) {
            declarations.addAll(setYaw.imu().dependencies());
            return;
        }
        if (action instanceof PathAction path) {
            declarations.add(path);
        } else if (action instanceof Actions.MotorNeutral motor) {
            declarations.add(motor.motor());
        } else if (action instanceof Actions.MotorPercent motor) {
            declarations.add(motor.motor());
        } else if (action instanceof Actions.DynamicMotorPercent motor) {
            declarations.add(motor.motor());
        } else if (action instanceof Actions.MotorVoltage motor) {
            declarations.add(motor.motor());
        } else if (action instanceof Actions.DynamicMotorVoltage motor) {
            declarations.add(motor.motor());
        } else if (action instanceof Actions.EncoderSetPosition encoder) {
            declarations.add(encoder.encoder());
        } else if (action instanceof CharacterizationRoutine.Test sysId) {
            declarations.add(sysId.routine().control());
            declarations.addAll(controlDeclarations(sysId.routine().control()));
        } else if (control(action) != null) {
            declarations.add(control(action));
            declarations.addAll(controlDeclarations(control(action)));
        }
        for (Action.Child child : action.children()) {
            collectActionDeclarations(child.action(), declarations);
        }
    }

    private static Set<Object> controlDeclarations(ControlBinding<?> control) {
        Set<Object> declarations = new LinkedHashSet<>();
        declarations.addAll(control.motors());
        if (control.feedback() != null) {
            declarations.addAll(control.feedback().dependencies());
        }
        if (control.sink() != null) {
            declarations.addAll(control.sink().dependencies());
        }
        declarations.addAll(control.dependencies());
        for (ControlLoop loop : control.loops()) {
            declarations.addAll(loop.dependencies());
        }
        return declarations;
    }

    private static ControlBinding<?> control(Action action) {
        if (action instanceof Actions.ControlNeutral control) {
            return control.control();
        }
        if (action instanceof Actions.ControlPercent control) {
            return control.control();
        }
        if (action instanceof Actions.DynamicControlPercent control) {
            return control.control();
        }
        if (action instanceof Actions.ControlVoltage control) {
            return control.control();
        }
        if (action instanceof Actions.DynamicControlVoltage control) {
            return control.control();
        }
        if (action instanceof CharacterizationRoutine.Voltage sysId) {
            return sysId.routine().control();
        }
        if (action instanceof Actions.ControlPosition control) {
            return control.control();
        }
        if (action instanceof Actions.DynamicControlPosition control) {
            return control.control();
        }
        if (action instanceof Actions.ControlVelocity control) {
            return control.control();
        }
        if (action instanceof Actions.DynamicControlVelocity control) {
            return control.control();
        }
        if (action instanceof InterpolatedControlAction interpolated) {
            return interpolated.control();
        }
        return null;
    }

    private boolean canResolveMotor(MotorDevice device) {
        try {
            actionContext.motor(device);
            return true;
        } catch (RuntimeException exception) {
            return false;
        }
    }

    private boolean canResolveEncoder(EncoderDevice device) {
        try {
            actionContext.encoder(device);
            return true;
        } catch (RuntimeException exception) {
            return false;
        }
    }

    private record OverlayActionContext(MappedActionContext registered, ActionContext fallback) implements ActionContext {
        private OverlayActionContext {
            registered = Objects.requireNonNull(registered, "registered");
            fallback = Objects.requireNonNull(fallback, "fallback");
        }

        @Override
        public EncoderHandle encoder(EncoderDevice ref) {
            if (registered.hasEncoder(ref)) {
                return registered.encoder(ref);
            }
            return fallback.encoder(ref);
        }

        @Override
        public MotorHandle motor(MotorDevice ref) {
            if (registered.hasMotor(ref)) {
                return registered.motor(ref);
            }
            return fallback.motor(ref);
        }

        @Override
        public ImuHandle imu(ImuDevice ref) {
            if (registered.hasImu(ref)) {
                return registered.imu(ref);
            }
            return fallback.imu(ref);
        }

        @Override
        public List<ActionContext.SoftwareMotorFollower> softwareFollowers(MotorDevice leader) {
            List<ActionContext.SoftwareMotorFollower> registeredFollowers = registered.softwareFollowers(leader);
            return registeredFollowers.isEmpty() ? fallback.softwareFollowers(leader) : registeredFollowers;
        }

        @Override
        public void hardwareFailure(Object declaration, RuntimeException exception) {
            fallback.hardwareFailure(declaration, exception);
        }
    }

    private record RequestTarget(Mechanism root, Mechanism owner) {
        private RequestTarget {
            root = Objects.requireNonNull(root, "root");
            owner = Objects.requireNonNull(owner, "owner");
        }
    }

    private record LeaseRegistration(Object runtimeKey, MechanismRuntime runtime) {
    }

    private record LeaseReservation(long recency, Set<Object> resources, boolean exclusive) {
        private LeaseReservation {
            resources = Set.copyOf(resources);
        }
    }

    private record CompiledAction(
            Map<Mechanism, CompiledPartition> partitions,
            Set<Object> resources,
            boolean exclusive) {
        private CompiledAction {
            partitions = Collections.unmodifiableMap(new IdentityHashMap<>(partitions));
            resources = Set.copyOf(resources);
        }
    }

    private record CompiledPartition(Action action, Set<Object> resources, boolean opaque) {
        private CompiledPartition {
            action = Objects.requireNonNull(action, "action");
            resources = Set.copyOf(resources);
        }
    }

    private static final class MemoryMotor implements MotorHandle {
        private final MotorDevice device;
        private double position;
        private double velocity;
        private ca.frc6390.athena.hardware.backend.MotorRuntimeConfig runtimeConfig;

        private MemoryMotor(MotorDevice device) {
            this.device = Objects.requireNonNull(device, "device");
            runtimeConfig = ca.frc6390.athena.hardware.backend.MotorRuntimeConfig.declared(device);
        }

        @Override
        public MotorDevice device() {
            return device;
        }

        @Override public boolean supportsRuntimeConfiguration() { return true; }
        @Override public void applyRuntimeConfiguration(
                ca.frc6390.athena.hardware.backend.MotorRuntimeConfig configuration) {
            runtimeConfig = Objects.requireNonNull(configuration, "configuration");
        }

        @Override
        public void setPercentOutput(double percent) {
            velocity = percent;
        }

        @Override
        public void setVoltage(double volts) {
            velocity = volts;
        }

        @Override
        public void setPositionTargetRotations(double rotations) {
            position = rotations;
        }

        @Override
        public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
            velocity = rotationsPerSecond;
        }

        @Override
        public double integratedPositionRotations() {
            return position;
        }

        @Override
        public double integratedVelocityRotationsPerSecond() {
            return velocity;
        }
    }

    private static final class MemoryEncoder implements EncoderHandle {
        private final EncoderDevice device;
        private double position;
        private double velocity;

        private MemoryEncoder(EncoderDevice device) {
            this.device = Objects.requireNonNull(device, "device");
        }

        @Override
        public EncoderDevice device() {
            return device;
        }

        @Override
        public double positionRotations() {
            return position;
        }

        @Override
        public double absolutePositionRotations() {
            return position;
        }

        @Override
        public double velocityRotationsPerSecond() {
            return velocity;
        }

        @Override
        public void setPositionRotations(double rotations) {
            position = Double.isFinite(rotations) ? rotations : 0.0;
        }

        @Override
        public boolean supportsPositionSetting() {
            return true;
        }
    }
}
