package ca.frc6390.athena.mechanism.core;

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
import java.util.function.BooleanSupplier;
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
    private final OutputResolver resolver;
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
        } else if (declaration instanceof ControlBinding control) {
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
            else if (owned instanceof ControlBinding control) motors.addAll(control.motors());
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
    public Set<ControlBinding> controlBindings() {
        Set<ControlBinding> controls = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
        for (Object declaration : graph.declarations(runtimes.keySet())) {
            if (declaration instanceof ControlBinding control) {
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
        } else if (declaration instanceof ControlBinding control) {
            control.motors().forEach(motor -> collectMotors(motor, motors));
        } else if (declaration instanceof Iterable<?> values) {
            values.forEach(value -> collectMotors(value, motors));
        }
    }

    private static void collectEncoders(Object declaration, Set<EncoderDevice> encoders) {
        if (declaration instanceof EncoderDevice encoder) {
            encoders.add(encoder);
        } else if (declaration instanceof ControlBinding control) {
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
        List<LeaseRegistration> registrations = new ArrayList<>(compiled.partitions().size());
        long recency = ++requestSequence;
        leaseReservations.put(key, new LeaseReservation(recency, compiled.resources()));
        for (Map.Entry<Mechanism, CompiledPartition> entry : compiled.partitions().entrySet()) {
            CompiledPartition partition = entry.getValue();
            MechanismRuntime runtime = runtimeFor(entry.getKey());
            Object runtimeKey = new Object();
            runtime.activateLease(
                    runtimeKey,
                    partition.action(),
                    recency,
                    partition.resources(),
                    actionRootPath(action) + " partition[" + graph.node(entry.getKey()).name() + "]");
            registrations.add(new LeaseRegistration(runtimeKey, runtime));
        }
        leaseTargets.put(key, List.copyOf(registrations));
        activeLeaseKeys.add(key);
    }

    private void releaseLease(Object key) {
        List<LeaseRegistration> registrations = leaseTargets.remove(key);
        leaseReservations.remove(key);
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
            CompiledPartition compiled = new CompiledPartition(partitioned, resources);
            compiledPartitions.put(entry.getKey(), compiled);
            allResources.addAll(resources);
        }
        return new CompiledAction(compiledPartitions, allResources);
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
        } else if (action instanceof Actions.Timeout timeout) {
            validateConcurrentResourceConflicts(timeout.action(), path + ".timeout", visited);
        } else if (action instanceof Actions.WithinTolerance within) {
            validateConcurrentResourceConflicts(within.action(), path + ".withinTolerance", visited);
        } else if (action instanceof Actions.VelocityContribution contribution) {
            validateConcurrentResourceConflicts(contribution.driveAction(), path + ".velocityDrive", visited);
        } else if (action instanceof Actions.Conditional conditional) {
            validateConcurrentResourceConflicts(conditional.action(), path + ".conditional.action", visited);
            validateConcurrentResourceConflicts(conditional.next(), path + ".conditional.next", visited);
        } else if (action instanceof Action.Conditional conditional) {
            validateConcurrentResourceConflicts(conditional.action(), path + ".conditional.action", visited);
            validateConcurrentResourceConflicts(conditional.next(), path + ".conditional.next", visited);
        } else if (action instanceof Actions.Then then) {
            validateConcurrentResourceConflicts(then.action(), path + ".then.action", visited);
            validateConcurrentResourceConflicts(then.next(), path + ".then.next", visited);
        } else if (action instanceof Action.Then then) {
            validateConcurrentResourceConflicts(then.action(), path + ".then.action", visited);
            validateConcurrentResourceConflicts(then.next(), path + ".then.next", visited);
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
        if (action instanceof Actions.Parallel parallel) {
            return parallel.Actions().stream().anyMatch(this::containsOpaqueRuntimeGraph);
        }
        if (action instanceof Actions.Race race) {
            return race.Actions().stream().anyMatch(this::containsOpaqueRuntimeGraph);
        }
        if (action instanceof Actions.Deadline deadline) {
            return deadline.Actions().stream().anyMatch(this::containsOpaqueRuntimeGraph);
        }
        if (action instanceof Actions.Sequence sequence) {
            return sequence.steps().stream().anyMatch(step -> containsOpaqueRuntimeGraph(step.action()))
                    || containsOpaqueRuntimeGraph(sequence.next());
        }
        if (action instanceof Actions.Cycle cycle) {
            return cycle.steps().stream().anyMatch(step -> containsOpaqueRuntimeGraph(step.action()));
        }
        if (action instanceof Actions.Choice choice) {
            return containsOpaqueRuntimeGraph(choice.active()) || containsOpaqueRuntimeGraph(choice.inactive());
        }
        if (action instanceof Actions.WhenBranch branch) return containsOpaqueRuntimeGraph(branch.active());
        if (action instanceof Actions.Timeout timeout) return containsOpaqueRuntimeGraph(timeout.action());
        if (action instanceof Actions.WithinTolerance within) return containsOpaqueRuntimeGraph(within.action());
        if (action instanceof Actions.Conditional conditional) {
            return containsOpaqueRuntimeGraph(conditional.action()) || containsOpaqueRuntimeGraph(conditional.next());
        }
        if (action instanceof Action.Conditional conditional) {
            return containsOpaqueRuntimeGraph(conditional.action()) || containsOpaqueRuntimeGraph(conditional.next());
        }
        if (action instanceof Actions.Then then) {
            return containsOpaqueRuntimeGraph(then.action()) || containsOpaqueRuntimeGraph(then.next());
        }
        if (action instanceof Action.Then then) {
            return containsOpaqueRuntimeGraph(then.action()) || containsOpaqueRuntimeGraph(then.next());
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
        if (declaration instanceof ControlBinding control && control.sink() != null) {
            resources.add(control.sink());
        }
        if (declaration instanceof MotorDevice motor) {
            resources.add(motor);
        } else if (declaration instanceof ControlBinding control) {
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
            path.markers().values().forEach(marker -> collectPathResources(marker, resources));
        } else if (action instanceof Actions.Parallel parallel) {
            parallel.Actions().forEach(child -> collectPathResources(child, resources));
        } else if (action instanceof Actions.Race race) {
            race.Actions().forEach(child -> collectPathResources(child, resources));
        } else if (action instanceof Actions.Deadline deadline) {
            deadline.Actions().forEach(child -> collectPathResources(child, resources));
        } else if (action instanceof Actions.Sequence sequence) {
            sequence.steps().forEach(step -> collectPathResources(step.action(), resources));
            collectPathResources(sequence.next(), resources);
        } else if (action instanceof Actions.Cycle cycle) {
            cycle.steps().forEach(step -> collectPathResources(step.action(), resources));
        } else if (action instanceof Actions.Choice choice) {
            collectPathResources(choice.active(), resources);
            collectPathResources(choice.inactive(), resources);
        } else if (action instanceof Actions.WhenBranch branch) {
            collectPathResources(branch.active(), resources);
        } else if (action instanceof Actions.Timeout timeout) {
            collectPathResources(timeout.action(), resources);
        } else if (action instanceof Actions.WithinTolerance within) {
            collectPathResources(within.action(), resources);
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
        Action safeAction = Objects.requireNonNull(action, "action");
        activateLease(safeAction, safeAction, true);
        return this;
    }

    /**
     * Cancels an action and any marker/start leases it launched. Resources no longer
     * driven by another lease are neutralized during the next output cycle.
     */
    public MechanismScheduler cancel(Action action) {
        cancelNested(Objects.requireNonNull(action, "action"),
                Collections.newSetFromMap(new IdentityHashMap<>()));
        return this;
    }

    /** Returns whether a requested action has an active, incomplete lease. */
    public boolean isRunning(Action action) {
        List<LeaseRegistration> registrations = leaseTargets.get(Objects.requireNonNull(action, "action"));
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
        Action safeAction = Objects.requireNonNull(action, "action");
        List<LeaseRegistration> registrations = leaseTargets.get(safeAction);
        if (registrations == null || registrations.isEmpty()) return completed(safeAction);
        for (LeaseRegistration registration : registrations) {
            if (!registration.runtime().leaseComplete(registration.runtimeKey())) return false;
        }
        return true;
    }

    private void cancelNested(Action action, Set<Action> visited) {
        if (action == null || !visited.add(action)) return;
        forgetCompletion(action);
        releaseLease(action);
        if (action instanceof PathAction path) {
            path.markers().values().forEach(marker -> cancelNested(marker, visited));
        } else if (action instanceof Actions.Sequence sequence) {
            sequence.steps().forEach(step -> cancelNested(step.action(), visited));
            cancelNested(sequence.next(), visited);
        } else if (action instanceof Actions.Parallel parallel) {
            parallel.Actions().forEach(child -> cancelNested(child, visited));
        } else if (action instanceof Actions.Race race) {
            race.Actions().forEach(child -> cancelNested(child, visited));
        } else if (action instanceof Actions.Deadline deadline) {
            deadline.Actions().forEach(child -> cancelNested(child, visited));
        } else if (action instanceof Actions.Choice choice) {
            cancelNested(choice.active(), visited); cancelNested(choice.inactive(), visited);
        } else if (action instanceof Actions.WhenBranch branch) {
            cancelNested(branch.active(), visited);
        } else if (action instanceof Actions.Timeout timeout) {
            cancelNested(timeout.action(), visited);
        } else if (action instanceof Actions.WithinTolerance within) {
            cancelNested(within.action(), visited);
        } else if (action instanceof Actions.VelocityContribution contribution) {
            cancelNested(contribution.driveAction(), visited);
        } else if (action instanceof Actions.Conditional conditional) {
            cancelNested(conditional.action(), visited); cancelNested(conditional.next(), visited);
        } else if (action instanceof Action.Conditional conditional) {
            cancelNested(conditional.action(), visited); cancelNested(conditional.next(), visited);
        } else if (action instanceof Actions.Then then) {
            cancelNested(then.action(), visited); cancelNested(then.next(), visited);
        } else if (action instanceof Action.Then then) {
            cancelNested(then.action(), visited); cancelNested(then.next(), visited);
        }
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
        sampleSignals();
        List<ResolvedOutput> outputs = new ArrayList<>(outputCapacityHint);
        Map<MechanismRuntime, Set<MotorDevice>> drivenByRuntime = drivenByRuntimeScratch;
        Set<MotorDevice> globallyDriven = globallyDrivenScratch;
        drivenByRuntime.clear();
        globallyDriven.clear();
        try {
            for (MechanismRuntime runtime : runtimes.values()) {
                runtime.runHooks(eventContext, false);
            }
            Map<Object, Long> reservations = globalLeaseReservations();
            for (MechanismRuntime runtime : runtimes.values()) {
                Set<MotorDevice> driven = runtime.periodicOutputsInto(safeMechanismContext, outputs, reservations);
                drivenByRuntime.put(runtime, driven);
                globallyDriven.addAll(driven);
            }
        } finally {
            digitalInputs.forEach(DigitalInputDevice::clearLatchedEdges);
            for (MechanismRuntime runtime : runtimes.values()) {
                runtime.finishOutputCycle(globallyDriven, drivenByRuntime.getOrDefault(runtime, Set.of()));
            }
            retireCompletedRequests();
        }
        outputCapacityHint = Math.max(outputCapacityHint, outputs.size());
        return outputs;
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
            for (Object resource : lease.resources()) {
                reservations.merge(resource, lease.recency(), Math::max);
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
        if (declaration instanceof ControlBinding control) {
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
        if (declarations.isEmpty() && runtimes.size() == 1) {
            Mechanism root = runtimes.keySet().iterator().next();
            return new RequestTarget(root, root);
        }
        return null;
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
        if (action instanceof Actions.Parallel parallel) {
            parallel.Actions().forEach(child -> collectActionDeclarations(child, declarations));
        } else if (action instanceof Actions.Computed computed) {
            for (Object owner : computed.ownership()) {
                if (owner instanceof Action ownedAction) collectActionDeclarations(ownedAction, declarations);
                else if (owner instanceof Iterable<?> values) {
                    values.forEach(value -> {
                        if (value instanceof Action ownedAction) collectActionDeclarations(ownedAction, declarations);
                        else declarations.add(value);
                    });
                } else declarations.add(owner);
            }
        } else if (action instanceof Actions.HardwareComputed computed) {
            declarations.addAll(computed.declarations());
        } else if (action instanceof Actions.VelocityContribution contribution) {
            collectActionDeclarations(contribution.driveAction(), declarations);
        } else if (action instanceof Actions.ImuYawMutation setYaw) {
            declarations.addAll(setYaw.imu().dependencies());
        } else if (action instanceof Actions.Race race) {
            race.Actions().forEach(child -> collectActionDeclarations(child, declarations));
        } else if (action instanceof Actions.Deadline deadline) {
            deadline.Actions().forEach(child -> collectActionDeclarations(child, declarations));
        } else if (action instanceof Actions.Sequence sequence) {
            sequence.steps().forEach(step -> collectActionDeclarations(step.action(), declarations));
            collectActionDeclarations(sequence.next(), declarations);
        } else if (action instanceof Actions.Cycle cycle) {
            cycle.steps().forEach(step -> collectActionDeclarations(step.action(), declarations));
        } else if (action instanceof Actions.Choice choice) {
            collectActionDeclarations(choice.active(), declarations);
            collectActionDeclarations(choice.inactive(), declarations);
        } else if (action instanceof Actions.WhenBranch branch) {
            collectActionDeclarations(branch.active(), declarations);
        } else if (action instanceof Actions.Timeout timeout) {
            collectActionDeclarations(timeout.action(), declarations);
        } else if (action instanceof Actions.WithinTolerance within) {
            collectActionDeclarations(within.action(), declarations);
        } else if (action instanceof Actions.Conditional conditional) {
            collectActionDeclarations(conditional.action(), declarations);
            collectActionDeclarations(conditional.next(), declarations);
        } else if (action instanceof Action.Conditional conditional) {
            collectActionDeclarations(conditional.action(), declarations);
            collectActionDeclarations(conditional.next(), declarations);
        } else if (action instanceof Actions.Then then) {
            collectActionDeclarations(then.action(), declarations);
            collectActionDeclarations(then.next(), declarations);
        } else if (action instanceof Action.Then then) {
            collectActionDeclarations(then.action(), declarations);
            collectActionDeclarations(then.next(), declarations);
        } else if (action instanceof PathAction path) {
            declarations.add(path);
            path.markers().values().forEach(marker -> collectActionDeclarations(marker, declarations));
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
        } else if (action instanceof Actions.ControlSysIdAction sysId) {
            declarations.add(sysId.routine().control());
            declarations.addAll(controlDeclarations(sysId.routine().control()));
        } else if (control(action) != null) {
            declarations.add(control(action));
            declarations.addAll(controlDeclarations(control(action)));
        }
    }

    private static Set<Object> controlDeclarations(ControlBinding control) {
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

    private static ControlBinding control(Action action) {
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
        if (action instanceof Actions.ControlSysIdVoltage sysId) {
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

    private record LeaseReservation(long recency, Set<Object> resources) {
        private LeaseReservation {
            resources = Set.copyOf(resources);
        }
    }

    private record CompiledAction(
            Map<Mechanism, CompiledPartition> partitions,
            Set<Object> resources) {
        private CompiledAction {
            partitions = Collections.unmodifiableMap(new IdentityHashMap<>(partitions));
            resources = Set.copyOf(resources);
        }
    }

    private record CompiledPartition(Action action, Set<Object> resources) {
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
