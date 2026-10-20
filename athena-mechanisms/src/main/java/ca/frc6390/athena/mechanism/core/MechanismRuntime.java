package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.RuntimeHardwareAccess;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.mechanism.staging.StageCaps;
import ca.frc6390.athena.hardware.runtime.HardwareCycleSnapshot;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.api.FailurePolicy;

/**
 * Minimal Athena-owned runtime loop for the new mechanism model.
 */
final class MechanismRuntime {
    private static final String MARKER_PREFIX = "pathMarker:";
    /** Absorbs tick-time rounding so a settle time of N control periods completes on the Nth cycle. */
    private static final double SETTLE_EPSILON_SECONDS = 1.0e-9;
    private final Mechanism mechanism;
    private final MechanismNode node;
    private final ActionContext actionContext;
    private final OutputResolver resolver;
    private final OutputApplier applier;
    private final HookRuntime hookRuntime;
    private final List<HookBinding> hookBindings;
    private final StateScheduler scheduler;
    private final Map<Object, ActiveLease> activeLeases = new IdentityHashMap<>();
    private final List<ActiveLease> leasesByRecency = new ArrayList<>();
    private Set<MotorDevice> previouslyDrivenMotors = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    private final List<CandidateOutput> candidateScratch = new ArrayList<>();
    private final List<CandidateOutput> candidatePool = new ArrayList<>();
    private final List<CandidateOutput> selectedScratch = new ArrayList<>();
    private final List<ResolvedOutput> resolvedScratch = new ArrayList<>();
    private final Map<Object, CandidateOutput> winnerScratch = new IdentityHashMap<>();
    private Set<MotorDevice> drivenScratch = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
    private final Map<PathAction, PathRuntime> pathRuntimes;
    private Runnable simulationStep = () -> {
    };
    private Action action;
    private double stateStartSeconds = Double.NaN;
    private double disabledSinceSeconds = Double.NaN;
    private int evaluationIndex = -1;
    private long actionRecency;
    private long localRecency;
    private MechanismTraceSnapshot traceSnapshot;
    private MechanismTraceLevel traceLevel = MechanismTraceLevel.OFF;
    private double tracePeriodSeconds = 0.10;
    private double lastTraceSeconds = Double.NEGATIVE_INFINITY;
    private LeaseResources leaseResources;
    private RuntimeHooks runtimeHooks;
    private MechanismContext evaluatedContext = MechanismContext.empty();
    private int reasonKind = -1;
    private Object reasonSubject;
    private Object reasonDetail;
    private int reasonNumber;
    private String reasonText = "Idle";
    private PowerRuntime electrical;
    private MechanismContext evaluatedTimedContext = MechanismContext.empty();
    private String lastFault = "";
    private boolean faultedThisCycle;

    /** Resolves the static resources of action subtrees for dynamic lease reservations. */
    interface LeaseResources {
        Set<Object> of(Action action);

        Set<Object> pathOwnership(PathAction path);
    }

    /** Scheduler callbacks used for spawn tracking and per-mechanism fault isolation. */
    interface RuntimeHooks {
        void enterLease(Object owner);

        void exitLease();

        /** Records a fault, or rethrows it when the scheduler does not isolate faults. */
        void fault(String location, RuntimeException exception);
    }

    private MechanismRuntime(
            MechanismNode node,
            ActionContext actionContext,
            OutputResolver resolver,
            Map<PathAction, PathRuntime> pathRuntimes,
            HookRuntime.LeaseController leaseController) {
        this.node = Objects.requireNonNull(node, "node");
        this.mechanism = Objects.requireNonNull(node.mechanism(), "mechanism");
        this.actionContext = Objects.requireNonNull(actionContext, "actionContext");
        this.resolver = Objects.requireNonNull(resolver, "resolver");
        this.applier = OutputApplier.using(actionContext, resolver.overrides());
        this.hookRuntime = new HookRuntime(leaseController, applier::resetControls, applier::encoderPositionSet);
        this.hookBindings = List.copyOf(node.hooks().values());
        this.pathRuntimes = Objects.requireNonNull(pathRuntimes, "pathRuntimes");
        this.scheduler = new StateScheduler(
                actionContext, pathRuntimes, resolver.overrides(), applier::encoderPositionSet);
        this.action = Actions.neutral();
        this.traceSnapshot = emptyTrace();
    }

    static MechanismRuntime of(Mechanism mechanism, ActionContext actionContext) {
        return new MechanismRuntime(
                MechanismIntrospector.inspect(mechanism),
                actionContext,
                OutputResolver.empty(),
                new HashMap<>(),
                null);
    }

    static MechanismRuntime of(
            Mechanism mechanism,
            ActionContext actionContext,
            OutputResolver resolver) {
        return new MechanismRuntime(
                MechanismIntrospector.inspect(mechanism), actionContext, resolver, new HashMap<>(), null);
    }

    static MechanismRuntime of(
            Mechanism mechanism,
            ActionContext actionContext,
            OutputResolver resolver,
            Map<PathAction, PathRuntime> pathRuntimes) {
        return new MechanismRuntime(
                MechanismIntrospector.inspect(mechanism), actionContext, resolver, pathRuntimes, null);
    }

    static MechanismRuntime of(
            MechanismNode node,
            ActionContext actionContext,
            OutputResolver resolver,
            Map<PathAction, PathRuntime> pathRuntimes) {
        return new MechanismRuntime(node, actionContext, resolver, pathRuntimes, null);
    }

    static MechanismRuntime of(
            MechanismNode node,
            ActionContext actionContext,
            OutputResolver resolver,
            Map<PathAction, PathRuntime> pathRuntimes,
            HookRuntime.LeaseController leaseController) {
        return new MechanismRuntime(node, actionContext, resolver, pathRuntimes, leaseController);
    }

    MechanismRuntime path(PathAction ref, PathRuntime runtime) {
        pathRuntimes.put(Objects.requireNonNull(ref, "ref"), Objects.requireNonNull(runtime, "runtime"));
        return this;
    }

    MechanismRuntime integration(LeaseResources resources, RuntimeHooks hooks) {
        this.leaseResources = resources;
        this.runtimeHooks = hooks;
        return this;
    }

    String name() {
        return node.name();
    }

    String lastFault() {
        return lastFault;
    }

    boolean faultedThisCycle() {
        return faultedThisCycle;
    }

    void markFault(String location, RuntimeException exception) {
        faultedThisCycle = true;
        lastFault = location + ": " + exception;
    }

    MechanismRuntime simulationStep(Runnable simulationStep) {
        this.simulationStep = simulationStep == null ? () -> {
        } : simulationStep;
        return this;
    }

    public Action action() {
        ActiveLease newest = tracedLease();
        return newest != null && newest.recency() > actionRecency ? newest.action : action;
    }

    MechanismTraceSnapshot traceSnapshot() {
        return traceSnapshot;
    }

    void traceLevel(MechanismTraceLevel level) {
        MechanismTraceLevel requested = Objects.requireNonNull(level, "level");
        if (traceLevel == requested) return;
        traceLevel = requested;
        applier.captureTrace(level == MechanismTraceLevel.CAPTURE);
        lastTraceSeconds = Double.NEGATIVE_INFINITY;
        if (level == MechanismTraceLevel.OFF) traceSnapshot = emptyTrace();
    }

    void tracePeriodSeconds(double periodSeconds) {
        if (!Double.isFinite(periodSeconds) || periodSeconds < 0.0) {
            throw new IllegalArgumentException("Trace period must be finite and non-negative.");
        }
        if (Double.compare(tracePeriodSeconds, periodSeconds) == 0) return;
        tracePeriodSeconds = periodSeconds;
        lastTraceSeconds = Double.NEGATIVE_INFINITY;
    }

    public void set(Action action) {
        set(action, ++localRecency);
    }

    void set(Action action, long recency) {
        this.action = Objects.requireNonNull(action, "action");
        this.actionRecency = recency;
        this.stateStartSeconds = Double.NaN;
        this.scheduler.reset();
    }

    void activateLease(
            Object key,
            Action action,
            long recency,
            Set<Object> reservedResources,
            String diagnosticLocation) {
        activateLease(key, action, recency, reservedResources, diagnosticLocation, null, false);
    }

    /**
     * Activates a lease. A narrowable lease reserves only resources of nodes that are active or
     * reachable soon (see {@link StateScheduler#collectLive}); otherwise it reserves its compiled set.
     */
    void activateLease(
            Object key,
            Action action,
            long recency,
            Set<Object> reservedResources,
            String diagnosticLocation,
            Object owner,
            boolean narrowable) {
        Objects.requireNonNull(key, "key");
        Objects.requireNonNull(action, "action");
        if (!activeLeases.containsKey(key)) {
            ActiveLease lease = new ActiveLease(
                    action, recency, actionContext, pathRuntimes, resolver.overrides(),
                    applier::encoderPositionSet, reservedResources, diagnosticLocation, owner, narrowable);
            activeLeases.put(key, lease);
            leasesByRecency.add(lease);
            leasesByRecency.sort(Comparator.comparingLong(ActiveLease::recency));
        }
    }

    /**
     * Ends running lifecycle Actions (onEnd(true)) in the base tree and every lease while the robot is
     * disabled. Other Actions keep their state; lifecycle Actions restart with onStart when enabled again.
     */
    private void interruptLifecycles() {
        scheduler.interruptLifecycles();
        for (ActiveLease lease : activeLeases.values()) {
            lease.scheduler.interruptLifecycles();
        }
    }

    void releaseLease(Object key) {
        ActiveLease removed = activeLeases.remove(key);
        if (removed != null) {
            int index = leasesByRecency.indexOf(removed);
            leasesByRecency.remove(index);
            // A lease evaluating this cycle may restart or cancel an earlier lease (for example
            // doOnce(() -> other.request())); keep the cursor on the next unevaluated lease.
            if (evaluationIndex >= 0 && index <= evaluationIndex) evaluationIndex--;
            removed.released = true;
            if (!removed.evaluating) removed.scheduler.reset();
        }
    }

    public List<ResolvedOutput> periodic(MechanismContext mechanismContext, EventContext eventContext) {
        List<ResolvedOutput> outputs = new ArrayList<>();
        periodicInto(mechanismContext, eventContext, outputs);
        return outputs;
    }

    void periodicInto(
            MechanismContext mechanismContext,
            EventContext eventContext,
            List<ResolvedOutput> outputs) {
        runHooks(eventContext);
        periodicOutputsInto(mechanismContext, outputs);
    }

    void runHooks(EventContext eventContext) {
        runHooks(eventContext, true);
    }

    void runHooks(EventContext eventContext, boolean finishEvents) {
        EventContext safeEventContext = eventContext == null ? EventContext.empty() : eventContext;
        hookRuntime.run(safeEventContext, actionContext, hookBindings, finishEvents);
    }

    void periodicOutputsInto(
            MechanismContext mechanismContext,
            List<ResolvedOutput> outputs) {
        evaluateCandidates(mechanismContext);
        Set<MotorDevice> driven = applyCandidates(outputs, leaseReservations());
        finishOutputCycle(driven, driven);
    }

    Set<MotorDevice> periodicOutputsInto(
            MechanismContext mechanismContext,
            List<ResolvedOutput> outputs,
            Map<Object, Long> reservations) {
        evaluateCandidates(mechanismContext);
        return applyCandidates(outputs, reservations);
    }

    /**
     * Evaluates the base action and every lease for this cycle without touching hardware. After
     * this call each lease's {@link #leaseReservation(Object) reservation} reflects its current
     * position in its action tree.
     */
    void evaluateCandidates(MechanismContext mechanismContext) {
        MechanismContext safeMechanismContext = mechanismContext == null ? MechanismContext.empty() : mechanismContext;
        evaluatedContext = safeMechanismContext;
        evaluatedTimedContext = safeMechanismContext;
        faultedThisCycle = false;
        List<CandidateOutput> candidates = candidateScratch;
        candidates.clear();
        if (!safeMechanismContext.enabled()) {
            if (Double.isNaN(disabledSinceSeconds)) disabledSinceSeconds = safeMechanismContext.nowSeconds();
            interruptLifecycles();
            return;
        }
        if (!Double.isNaN(disabledSinceSeconds)) {
            // Action timers do not advance while disabled: a wait, timeout, or forTime resumes
            // with the time it had left instead of completing on the first enabled cycle.
            shiftClocks(safeMechanismContext.nowSeconds() - disabledSinceSeconds);
            disabledSinceSeconds = Double.NaN;
        }
        if (Double.isNaN(stateStartSeconds)) {
            stateStartSeconds = safeMechanismContext.nowSeconds();
        }
        MechanismContext timedContext = withTimeInState(
                safeMechanismContext,
                safeMechanismContext.nowSeconds() - stateStartSeconds);
        try {
            Action updated = mechanism.update(action, timedContext);
            if (updated != action) {
                action = Objects.requireNonNull(updated, "updated");
                stateStartSeconds = safeMechanismContext.nowSeconds();
                scheduler.reset();
                timedContext = withTimeInState(safeMechanismContext, 0.0);
            }
            addCandidates(
                    candidates,
                    "base",
                    actionRecency,
                    scheduler.evaluate(action, timedContext),
                    null,
                    baseActionLocation(action));
        } catch (RuntimeException exception) {
            truncate(candidates, 0);
            reportFault(baseActionLocation(action), exception);
        }
        evaluatedTimedContext = timedContext;
        try {
            for (evaluationIndex = 0; evaluationIndex < leasesByRecency.size(); evaluationIndex++) {
                ActiveLease lease = leasesByRecency.get(evaluationIndex);
                if (lease.scheduler().complete()) {
                    lease.refreshReservation(leaseResources);
                    continue;
                }
                int firstCandidate = candidates.size();
                if (runtimeHooks != null) runtimeHooks.enterLease(lease.owner());
                lease.evaluating = true;
                try {
                    StateScheduler.Result result = lease.evaluate(safeMechanismContext);
                    if (!lease.released) {
                        addCandidates(
                                candidates,
                                "lease",
                                lease.recency(),
                                result,
                                lease.reservedResources(),
                                lease.diagnosticLocation());
                    }
                } catch (RuntimeException exception) {
                    truncate(candidates, firstCandidate);
                    if (!lease.released) reportFault(lease.diagnosticLocation(), exception);
                } finally {
                    lease.evaluating = false;
                    if (runtimeHooks != null) runtimeHooks.exitLease();
                }
                if (lease.released) {
                    // Cancelled or restarted while it evaluated; its replacement (if any) runs instead.
                    lease.scheduler().reset();
                    continue;
                }
                lease.refreshReservation(leaseResources);
            }
        } finally {
            evaluationIndex = -1;
        }
    }

    private void shiftClocks(double pausedSeconds) {
        if (!(pausedSeconds > 0.0)) return;
        if (!Double.isNaN(stateStartSeconds)) stateStartSeconds += pausedSeconds;
        scheduler.shift(pausedSeconds);
        for (ActiveLease lease : leasesByRecency) lease.shift(pausedSeconds);
    }

    /** Arbitrates the candidates from {@link #evaluateCandidates} and applies the winners. */
    /** Installs the staging caps for the coming output cycle. */
    void stageCaps(
            Map<ControlBinding<?>, StageCaps> caps,
            Map<ControlBinding<?>, Enum<?>> stages) {
        applier.stageCaps(caps, stages);
    }

    /** Installs the power pass whose per-motor thermal and budget caps explain this mechanism's outputs. */
    void electrical(PowerRuntime power) {
        electrical = power;
        applier.electrical(power);
    }

    Set<MotorDevice> applyCandidates(List<ResolvedOutput> outputs, Map<Object, Long> reservations) {
        Objects.requireNonNull(outputs, "outputs");
        Objects.requireNonNull(reservations, "reservations");
        MechanismContext safeMechanismContext = evaluatedContext;
        if (!safeMechanismContext.enabled()) {
            setReason(REASON_DISABLED, null, null, 0);
            applier.stopAll();
            previouslyDrivenMotors.clear();
            simulationStep.run();
            if (!traceDue(safeMechanismContext.nowSeconds())) return Set.of();
            ActiveLease tracedLease = tracedLease();
            Action tracedAction = tracedLease == null ? action : tracedLease.action();
            StateScheduler tracedScheduler = tracedLease == null ? scheduler : tracedLease.scheduler();
            traceSnapshot = new MechanismTraceSnapshot(
                    node.name(),
                    safeMechanismContext.nowSeconds(),
                    0.0,
                    false,
                    actionName(tracedAction),
                    typeName(tracedAction),
                    tracedScheduler.scheduledActionType(),
                    tracedScheduler.step(),
                    tracedScheduler.complete(),
                    activeLeases.size(),
                    List.of(),
                    List.of(),
                    traceLevel == MechanismTraceLevel.CAPTURE ? motorTraces(Map.of()) : List.of(),
                    traceLevel == MechanismTraceLevel.CAPTURE ? hookTraces() : List.of());
            lastTraceSeconds = safeMechanismContext.nowSeconds();
            return Set.of();
        }
        MechanismContext timedContext = evaluatedTimedContext;
        List<CandidateOutput> candidates = candidateScratch;
        List<CandidateOutput> selected = arbitrate(candidates, reservations);
        Set<MotorDevice> drivenNow = drivenScratch;
        drivenNow.clear();
        applier.beginCycle();
        try {
            for (int candidateIndex = 0; candidateIndex < selected.size(); candidateIndex++) {
                CandidateOutput candidate = selected.get(candidateIndex);
                try {
                    applier.apply(candidate.output(), candidate.context());
                } catch (RuntimeException exception) {
                    applier.abandon(candidate.output().request());
                    reportFault(candidate.location(), exception);
                    continue;
                }
                outputs.add(candidate.output());
                addMotors(drivenNow, candidate.output().request());
            }
            // After the apply loop, so the explanation sees the constraints and caps that acted on this cycle.
            updateReason(candidates, selected, reservations);
        } finally {
            applier.endCycle();
        }
        // Controls held after release (HOLD / DECELERATE) keep their motors driven without a lease.
        drivenNow.addAll(applier.heldMotors());
        simulationStep.run();
        if (traceDue(timedContext.nowSeconds())) {
            traceSnapshot = buildTrace(timedContext, candidates, selected, reservations);
            lastTraceSeconds = timedContext.nowSeconds();
        }
        return drivenNow;
    }

    /** Neutralizes every output of this mechanism after an unrecoverable evaluation fault. */
    Set<MotorDevice> neutralizeAfterFault() {
        candidateScratch.clear();
        drivenScratch.clear();
        try {
            applier.stopAll();
        } catch (RuntimeException ignored) {
            // Best effort: the scheduler still neutralizes previously driven motors.
        }
        return drivenScratch;
    }

    /** Returns the resources a lease currently reserves against older leases. */
    Set<Object> leaseReservation(Object key) {
        ActiveLease lease = activeLeases.get(key);
        return lease == null ? Set.of() : lease.currentReservation();
    }

    private void reportFault(String location, RuntimeException exception) {
        faultedThisCycle = true;
        lastFault = location + ": " + exception;
        if (runtimeHooks == null || FaultIsolation.isPanic(exception)) {
            throw exception;
        }
        runtimeHooks.fault(node.name() + " " + location, exception);
    }

    private static void truncate(List<CandidateOutput> candidates, int size) {
        while (candidates.size() > size) {
            candidates.remove(candidates.size() - 1);
        }
    }

    private boolean traceDue(double nowSeconds) {
        if (traceLevel == MechanismTraceLevel.OFF) return false;
        double elapsed = nowSeconds - lastTraceSeconds;
        return tracePeriodSeconds == 0.0 || elapsed < 0.0 || elapsed + 1.0e-9 >= tracePeriodSeconds;
    }

    void finishOutputCycle(Set<MotorDevice> globallyDriven, Set<MotorDevice> locallyDriven) {
        Objects.requireNonNull(globallyDriven, "globallyDriven");
        Objects.requireNonNull(locallyDriven, "locallyDriven");
        for (MotorDevice motor : previouslyDrivenMotors) {
            if (!globallyDriven.contains(motor)) {
                try {
                    actionContext.motor(motor).stop();
                } catch (RuntimeException exception) {
                    if (motor.failurePolicy() == FailurePolicy.PANIC) throw FaultIsolation.panic(exception);
                    actionContext.hardwareFailure(motor, exception);
                }
                motor.recordCommand(ca.frc6390.athena.hardware.signal.MotorCommandSnapshot.neutral());
            }
        }
        if (locallyDriven == drivenScratch) {
            Set<MotorDevice> reusable = previouslyDrivenMotors;
            previouslyDrivenMotors = drivenScratch;
            drivenScratch = reusable;
        } else {
            previouslyDrivenMotors.clear();
            previouslyDrivenMotors.addAll(locallyDriven);
        }
    }

    private void addCandidates(
            List<CandidateOutput> candidates,
            String source,
            long recency,
            StateScheduler.Result active,
            Set<Object> reservedResources,
            String diagnosticLocation) {
        resolvedScratch.clear();
        resolver.resolveInto(mechanism, active.action(), active.context(), resolvedScratch);
        validateNoDuplicateOutputs(resolvedScratch, source, active.action(), diagnosticLocation);
        for (ResolvedOutput output : resolvedScratch) {
            validateReservedResources(output.request(), reservedResources);
            int order = candidates.size();
            CandidateOutput candidate;
            if (order < candidatePool.size()) {
                candidate = candidatePool.get(order);
                candidate.reset(output, active.context(), source, recency, order, diagnosticLocation);
            } else {
                candidate = new CandidateOutput(
                        output, active.context(), source, recency, order, diagnosticLocation);
                candidatePool.add(candidate);
            }
            candidates.add(candidate);
        }
    }

    private static void validateNoDuplicateOutputs(
            List<ResolvedOutput> outputs,
            String source,
            Action scheduledOutput,
            String diagnosticLocation) {
        Map<Object, Integer> claimed = new LinkedHashMap<>();
        for (int outputIndex = 0; outputIndex < outputs.size(); outputIndex++) {
            ResolvedOutput output = outputs.get(outputIndex);
            OutputRequest request = output.request();
            for (int index = 0; index < motorCount(request); index++) {
                requireUnclaimedOutputResource(
                        claimed, motorAt(request, index), outputs, outputIndex,
                        source, scheduledOutput, diagnosticLocation);
            }
            if (request.control() != null && request.control().sink() != null) {
                requireUnclaimedOutputResource(
                        claimed, request.control().sink(), outputs, outputIndex,
                        source, scheduledOutput, diagnosticLocation);
            }
        }
    }

    private static void requireUnclaimedOutputResource(
            Map<Object, Integer> claimed,
            Object resource,
            List<ResolvedOutput> outputs,
            int outputIndex,
            String source,
            Action scheduledOutput,
            String diagnosticLocation) {
        Integer existingIndex = claimed.putIfAbsent(resource, outputIndex);
        if (existingIndex != null) {
            throw new IllegalStateException(
                    "Runtime output conflict at " + diagnosticLocation + " while evaluating "
                            + source + " action " + actionType(scheduledOutput)
                            + ": output[" + existingIndex + "] " + outputDescription(outputs.get(existingIndex))
                            + " and output[" + outputIndex + "] " + outputDescription(outputs.get(outputIndex))
                            + " both claim " + outputResourceDescription(resource)
                            + ". Put them in a sequence or remove one of the duplicate outputs.");
        }
    }

    private String baseActionLocation(Action baseAction) {
        List<String> names = new ArrayList<>();
        for (Map.Entry<String, Action> entry : node.Actions().entrySet()) {
            if (entry.getValue() == baseAction) names.add(node.name() + ".actions[\"" + entry.getKey() + "\"]");
        }
        if (!names.isEmpty()) return String.join(" or ", names);
        return node.name() + ".baseAction<" + actionType(baseAction) + ">";
    }

    private static String actionType(Action action) {
        String type = action.getClass().getSimpleName();
        return type.isBlank() ? action.getClass().getName() : type;
    }

    private static String outputDescription(ResolvedOutput resolved) {
        Output output = resolved.output();
        String type = output.getClass().getSimpleName();
        return type.isBlank() ? output.getClass().getName() : type;
    }

    private static String outputResourceDescription(Object resource) {
        if (resource instanceof MotorDevice motor) return "motor '" + motor.defaultName() + "'";
        return "resource '" + resource + "'";
    }

    private static void validateReservedResources(OutputRequest request, Set<Object> reservedResources) {
        if (reservedResources == null) {
            return;
        }
        for (int index = 0; index < motorCount(request); index++) {
            MotorDevice motor = motorAt(request, index);
            if (!reservedResources.contains(motor)) {
                throw new IllegalStateException(
                        "Action emitted motor '" + motor.defaultName()
                                + "' outside its compiled resource graph. Add it to the computed action ownership.");
            }
        }
        ControlBinding<?> control = request.control();
        if (control != null && control.sink() != null && !reservedResources.contains(control.sink())) {
            throw new IllegalStateException(
                    "Action emitted a control sink outside its compiled resource graph. "
                            + "Add it to the computed action ownership.");
        }
    }

    /** Returns the one-line explanation of this mechanism's output decision for the last applied cycle. */
    String reason() {
        return reasonText;
    }

    /**
     * Summarises why outputs are or are not applied. The first matching line wins, highest priority first:
     * Fault, Disabled, Blocked (a constraint rejected the target), Starved (a newer lease reserves a motor),
     * Superseded (a candidate lost arbitration), Guard (output clipped at a range edge), Stage (a staging
     * stage caps an applied output), Thermal derate, Power cap, Running, Idle.
     *
     * <p>Text is rebuilt only when the decision's subjects change, so an unchanged state allocates nothing.
     */
    private void updateReason(
            List<CandidateOutput> candidates,
            List<CandidateOutput> selected,
            Map<Object, Long> reservations) {
        if (faultedThisCycle) {
            setReason(REASON_FAULT, null, lastFault, 0);
            return;
        }
        String blocked = applier.blockedBy();
        if (!blocked.isEmpty()) {
            setReason(REASON_BLOCKED, blocked, null, 0);
            return;
        }
        for (int candidateIndex = 0; candidateIndex < candidates.size(); candidateIndex++) {
            CandidateOutput candidate = candidates.get(candidateIndex);
            if (selected.contains(candidate)) continue;
            OutputRequest request = candidate.output().request();
            for (int motorIndex = 0; motorIndex < motorCount(request); motorIndex++) {
                MotorDevice motor = motorAt(request, motorIndex);
                long reserved = reservations.getOrDefault(motor, Long.MIN_VALUE);
                if (candidate.recency() < reserved && !drivenBy(motor, selected)) {
                    setReason(REASON_STARVED, leaseWithRecency(reserved), motor, 0);
                    return;
                }
            }
        }
        if (selected.isEmpty()) {
            setReason(candidates.isEmpty() ? REASON_IDLE : REASON_SUPERSEDED, null, null, 0);
            return;
        }
        String guard = applier.guardEdge();
        if (!guard.isEmpty()) {
            setReason(REASON_GUARD, guard, null, 0);
            return;
        }
        Enum<?> stage = applier.activeStage();
        if (stage != null) {
            setReason(REASON_STAGE, stage, null, 0);
            return;
        }
        int thermal = -1;
        int power = -1;
        int unappliedCap = -1;
        if (electrical != null) {
            for (int candidateIndex = 0; candidateIndex < selected.size(); candidateIndex++) {
                OutputRequest request = selected.get(candidateIndex).output().request();
                for (int motorIndex = 0; motorIndex < motorCount(request); motorIndex++) {
                    MotorDevice motor = motorAt(request, motorIndex);
                    int derate = electrical.thermalDeratePercent(motor);
                    if (derate >= 0 && (thermal < 0 || derate < thermal)) thermal = derate;
                    int cap = electrical.powerCapAmps(motor);
                    if (cap < 0) continue;
                    int unapplied = applier.deviceLoopUnappliedCapAmps(request.control());
                    if (unapplied >= 0) {
                        if (unappliedCap < 0 || unapplied < unappliedCap) unappliedCap = unapplied;
                    } else if (power < 0 || cap < power) {
                        power = cap;
                    }
                }
            }
        }
        if (thermal >= 0) {
            setReason(REASON_THERMAL, null, null, thermal);
            return;
        }
        if (power >= 0) {
            setReason(REASON_POWER, null, null, power);
            return;
        }
        if (unappliedCap >= 0) {
            setReason(REASON_POWER_UNAPPLIED, null, null, unappliedCap);
            return;
        }
        CandidateOutput winner = selected.get(0);
        Object subject = "base".equals(winner.source()) ? action : leaseActionWithRecency(winner.recency());
        setReason(REASON_RUNNING, subject, null, 0);
    }

    private static boolean drivenBy(MotorDevice motor, List<CandidateOutput> selected) {
        for (int candidateIndex = 0; candidateIndex < selected.size(); candidateIndex++) {
            OutputRequest request = selected.get(candidateIndex).output().request();
            for (int motorIndex = 0; motorIndex < motorCount(request); motorIndex++) {
                if (motorAt(request, motorIndex) == motor) return true;
            }
        }
        return false;
    }

    private ActiveLease leaseWithRecency(long recency) {
        for (int index = 0; index < leasesByRecency.size(); index++) {
            ActiveLease lease = leasesByRecency.get(index);
            if (lease.recency() == recency) return lease;
        }
        return null;
    }

    private Action leaseActionWithRecency(long recency) {
        ActiveLease lease = leaseWithRecency(recency);
        return lease == null ? action : lease.action();
    }

    private void setReason(int kind, Object subject, Object detail, int number) {
        if (kind == reasonKind && sameReasonValue(subject, reasonSubject) && number == reasonNumber
                && sameReasonValue(detail, reasonDetail)) {
            return;
        }
        reasonKind = kind;
        reasonSubject = subject;
        reasonDetail = detail;
        reasonNumber = number;
        reasonText = describeReason(kind, subject, detail, number);
    }

    private static boolean sameReasonValue(Object value, Object previous) {
        if (value == previous) return true;
        return value instanceof String text && text.equals(previous);
    }

    private String describeReason(int kind, Object subject, Object detail, int number) {
        return switch (kind) {
            case REASON_DISABLED -> "Disabled";
            case REASON_FAULT -> "Fault: " + detail;
            case REASON_BLOCKED -> "Blocked: " + subject + " constraint rejected target";
            case REASON_STARVED -> "Starved: newer lease "
                    + (subject instanceof ActiveLease lease ? actionName(lease.action()) : "unknown")
                    + " reserves " + outputResourceDescription(detail);
            case REASON_SUPERSEDED -> "Superseded by newer action on shared resource";
            case REASON_GUARD -> "Guard: at " + subject + " edge";
            case REASON_STAGE -> "Stage " + ((Enum<?>) subject).name();
            case REASON_THERMAL -> "Thermal derate " + number + "%";
            case REASON_POWER -> "Power cap " + number + " A";
            case REASON_POWER_UNAPPLIED -> "Power cap " + number + " A not applied (device loop)";
            case REASON_RUNNING -> "Running: " + (subject instanceof Action running ? actionName(running) : "unknown");
            default -> "Idle";
        };
    }

    private static final int REASON_IDLE = 0;
    private static final int REASON_DISABLED = 1;
    private static final int REASON_FAULT = 2;
    private static final int REASON_STARVED = 3;
    private static final int REASON_RUNNING = 4;
    private static final int REASON_SUPERSEDED = 5;
    private static final int REASON_BLOCKED = 6;
    private static final int REASON_GUARD = 7;
    private static final int REASON_STAGE = 8;
    private static final int REASON_THERMAL = 9;
    private static final int REASON_POWER = 10;
    private static final int REASON_POWER_UNAPPLIED = 11;

    private MechanismTraceSnapshot buildTrace(
            MechanismContext context,
            List<CandidateOutput> candidates,
            List<CandidateOutput> selected,
            Map<Object, Long> reservations) {
        ActiveLease tracedLease = tracedLease();
        Action tracedAction = tracedLease == null ? action() : tracedLease.action();
        StateScheduler tracedScheduler = tracedLease == null ? scheduler : tracedLease.scheduler();
        if (traceLevel == MechanismTraceLevel.SUMMARY) {
            return new MechanismTraceSnapshot(
                    node.name(), context.nowSeconds(),
                    tracedLease == null ? context.timeInStateSeconds() : tracedLease.timeInState(context.nowSeconds()),
                    context.enabled(), actionName(tracedAction), typeName(tracedAction),
                    tracedScheduler.scheduledActionType(), tracedScheduler.step(), tracedScheduler.complete(),
                    activeLeases.size(), List.of(), List.of(), List.of(), List.of());
        }
        Set<CandidateOutput> winners = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
        winners.addAll(selected);
        List<MechanismTraceSnapshot.ActionCandidate> candidateTraces = candidates.stream()
                .map(candidate -> new MechanismTraceSnapshot.ActionCandidate(
                        candidate.source(),
                        typeName(candidate.output().output()),
                        candidate.recency(),
                        candidate.order(),
                        winners.contains(candidate),
                        candidateDecision(candidate, winners, reservations),
                        outputMode(candidate.output().output()),
                        outputValue(candidate.output().output()),
                        candidate.output().request().motors().stream().map(MotorDevice::defaultName).toList()))
                .toList();
        Map<MotorDevice, CandidateOutput> commands = new LinkedHashMap<>();
        selected.forEach(candidate -> candidate.output().request().motors().forEach(motor -> commands.put(motor, candidate)));
        List<MechanismTraceSnapshot.Motor> motors = motorTraces(commands);
        return new MechanismTraceSnapshot(
                node.name(),
                context.nowSeconds(),
                tracedLease == null ? context.timeInStateSeconds() : tracedLease.timeInState(context.nowSeconds()),
                context.enabled(),
                actionName(tracedAction),
                typeName(tracedAction),
                tracedScheduler.scheduledActionType(),
                tracedScheduler.step(),
                tracedScheduler.complete(),
                activeLeases.size(),
                candidateTraces,
                applier.controlTraces(),
                motors,
                hookTraces());
    }

    private List<MechanismTraceSnapshot.Motor> motorTraces(Map<MotorDevice, CandidateOutput> commands) {
        Set<MotorDevice> motors = new LinkedHashSet<>();
        node.declarations().values().forEach(declaration -> {
            if (declaration instanceof MotorDevice motor) {
                motors.add(motor);
            } else if (declaration instanceof ControlBinding<?> control) {
                motors.addAll(control.motors());
            }
        });
        motors.addAll(commands.keySet());
        List<MechanismTraceSnapshot.Motor> traces = new ArrayList<>(motors.size());
        HardwareCycleSnapshot hardwareSnapshot = actionContext instanceof HardwareGraph graph
                ? graph.cycleSnapshot()
                : null;
        for (MotorDevice motor : motors) {
            MotorHandle handle = actionContext.motor(motor);
            CandidateOutput candidate = commands.get(motor);
            OutputApplier.AppliedMotorCommand applied = applier.appliedMotorCommand(motor);
            String commandMode = "neutral";
            double commandValue = 0.0;
            if (applied != null) {
                commandMode = applied.mode();
                commandValue = applied.value();
            } else if (candidate != null) {
                commandMode = outputMode(candidate.output().output());
                commandValue = outputValue(candidate.output().output());
            }
            HardwareCycleSnapshot.MotorInput input = hardwareSnapshot == null
                    ? null
                    : hardwareSnapshot.motor(motor).orElse(null);
            traces.add(new MechanismTraceSnapshot.Motor(
                    motor.defaultName(),
                    commandMode,
                    commandValue,
                    input == null ? read(handle::integratedPositionRotations) : input.positionRotations(),
                    input == null ? read(handle::integratedVelocityRotationsPerSecond) : input.velocityRotationsPerSecond(),
                    input == null ? read(handle::appliedVoltage) : input.appliedVoltage(),
                    input == null ? read(handle::supplyCurrentAmps) : input.supplyCurrentAmps(),
                    input == null ? read(handle::statorCurrentAmps) : input.statorCurrentAmps(),
                    resolver.overrides().disabled(motor),
                    input == null || input.connected(),
                    input == null ? "" : input.failure()));
        }
        return List.copyOf(traces);
    }

    private String candidateDecision(
            CandidateOutput candidate,
            Set<CandidateOutput> winners,
            Map<Object, Long> reservations) {
        if (winners.contains(candidate)) return "selected";
        OutputRequest request = candidate.output().request();
        for (MotorDevice motor : request.motors()) {
            if (candidate.recency() < reservations.getOrDefault(motor, Long.MIN_VALUE)) {
                return "rejected: resource reserved by newer action";
            }
        }
        if (request.control() != null && request.control().sink() != null
                && candidate.recency() < reservations.getOrDefault(request.control().sink(), Long.MIN_VALUE)) {
            return "rejected: control sink reserved by newer action";
        }
        return "rejected: superseded by newer action on shared resource";
    }

    private ActiveLease tracedLease() {
        return leasesByRecency.isEmpty() ? null : leasesByRecency.get(leasesByRecency.size() - 1);
    }

    private List<MechanismTraceSnapshot.Hook> hookTraces() {
        List<MechanismTraceSnapshot.Hook> traces = new ArrayList<>();
        node.hooks().forEach((name, hook) -> {
            HookRuntime.HookStatus status = hookRuntime.status(hook);
            traces.add(new MechanismTraceSnapshot.Hook(
                    name,
                    status.sourceActive(),
                    status.active(),
                    status.triggeredThisCycle()));
        });
        return List.copyOf(traces);
    }

    private MechanismTraceSnapshot emptyTrace() {
        return new MechanismTraceSnapshot(
                node.name(), 0.0, 0.0, false, "", "", "", -1, false, 0,
                List.of(), List.of(), List.of(), List.of());
    }

    private String actionName(Action selected) {
        for (Map.Entry<String, Action> entry : node.Actions().entrySet()) {
            if (entry.getValue() == selected) {
                return entry.getKey();
            }
        }
        return typeName(selected);
    }

    private static String typeName(Object value) {
        if (value == null) {
            return "";
        }
        String name = value.getClass().getSimpleName();
        return name.isBlank() ? value.getClass().getName() : name;
    }

    private static String outputMode(Output output) {
        if (output instanceof Output.Percent) return "percent";
        if (output instanceof Output.Voltage) return "voltage";
        if (output instanceof Output.Position) return "position";
        if (output instanceof Output.Velocity) return "velocity";
        if (output instanceof Output.Fault) return "fault";
        return "neutral";
    }

    private static double outputValue(Output output) {
        if (output instanceof Output.Percent percent) return percent.percent();
        if (output instanceof Output.Voltage voltage) return voltage.volts();
        if (output instanceof Output.Position position) return position.position();
        if (output instanceof Output.Velocity velocity) return velocity.velocity();
        return 0.0;
    }

    private static double read(DoubleRead read) {
        try {
            return read.get();
        } catch (UnsupportedOperationException ignored) {
            return Double.NaN;
        }
    }

    @FunctionalInterface
    private interface DoubleRead {
        double get();
    }

    private Map<Object, Long> leaseReservations() {
        Map<Object, Long> reservations = new LinkedHashMap<>();
        for (ActiveLease lease : leasesByRecency) {
            if (lease.scheduler().complete()) {
                continue;
            }
            for (Object resource : lease.currentReservation()) {
                reservations.merge(resource, lease.recency(), Math::max);
            }
        }
        return reservations;
    }

    boolean leaseComplete(Object key) {
        ActiveLease lease = activeLeases.get(key);
        return lease != null && lease.scheduler().complete();
    }

    private List<CandidateOutput> arbitrate(
            List<CandidateOutput> candidates,
            Map<Object, Long> reservations) {
        Map<Object, CandidateOutput> winners = winnerScratch;
        winners.clear();
        for (int index = 0; index < candidates.size(); index++) {
            CandidateOutput candidate = candidates.get(index);
            OutputRequest request = candidate.output().request();
            int motorCount = motorCount(request);
            for (int motorIndex = 0; motorIndex < motorCount; motorIndex++) {
                selectWinner(motorAt(request, motorIndex), candidate, reservations, winners);
            }
            if (request.control() != null && request.control().sink() != null) {
                selectWinner(request.control().sink(), candidate, reservations, winners);
            }
        }
        List<CandidateOutput> selected = selectedScratch;
        selected.clear();
        for (int index = 0; index < candidates.size(); index++) {
            CandidateOutput candidate = candidates.get(index);
            if (winsAllResources(candidate, winners)) {
                selected.add(candidate);
            }
        }
        return selected;
    }

    private static void selectWinner(
            Object resource,
            CandidateOutput candidate,
            Map<Object, Long> reservations,
            Map<Object, CandidateOutput> winners) {
        Long reservation = reservations.get(resource);
        if (reservation != null && candidate.recency() < reservation.longValue()) {
            return;
        }
        CandidateOutput current = winners.get(resource);
        if (current == null || candidate.newerThan(current)) {
            winners.put(resource, candidate);
        }
    }

    private static boolean winsAllResources(
            CandidateOutput candidate,
            Map<Object, CandidateOutput> winners) {
        OutputRequest request = candidate.output().request();
        int motorCount = motorCount(request);
        boolean hasResource = motorCount > 0;
        for (int index = 0; index < motorCount; index++) {
            if (winners.get(motorAt(request, index)) != candidate) return false;
        }
        if (request.control() != null && request.control().sink() != null) {
            hasResource = true;
            if (winners.get(request.control().sink()) != candidate) return false;
        }
        return hasResource;
    }

    private static int motorCount(OutputRequest request) {
        return request.control() != null ? request.control().motors().size() : request.motor() == null ? 0 : 1;
    }

    private static MotorDevice motorAt(OutputRequest request, int index) {
        if (request.control() != null) return request.control().motors().get(index);
        if (index == 0 && request.motor() != null) return request.motor();
        throw new IndexOutOfBoundsException(index);
    }

    private static void addMotors(Set<MotorDevice> destination, OutputRequest request) {
        for (int index = 0; index < motorCount(request); index++) {
            destination.add(motorAt(request, index));
        }
    }

    private static final class CandidateOutput {
        private ResolvedOutput output;
        private MechanismContext context;
        private String source;
        private long recency;
        private int order;
        private String location;

        private CandidateOutput(
                ResolvedOutput output,
                MechanismContext context,
                String source,
                long recency,
                int order,
                String location) {
            reset(output, context, source, recency, order, location);
        }

        private void reset(
                ResolvedOutput output,
                MechanismContext context,
                String source,
                long recency,
                int order,
                String location) {
            this.output = output;
            this.context = context;
            this.source = source;
            this.recency = recency;
            this.order = order;
            this.location = location;
        }

        private String location() { return location; }

        private ResolvedOutput output() { return output; }
        private MechanismContext context() { return context; }
        private String source() { return source; }
        private long recency() { return recency; }
        private int order() { return order; }

        private boolean newerThan(CandidateOutput other) {
            return recency > other.recency || recency == other.recency && order > other.order;
        }
    }

    static final class ScheduledOutputs implements Action {
        private final List<Action> actions;
        private final boolean laterWins;

        private ScheduledOutputs(List<Action> actions) {
            this(actions, false);
        }

        private ScheduledOutputs(List<Action> actions, boolean laterWins) {
            this.actions = actions;
            this.laterWins = laterWins;
        }

        List<Action> actions() {
            return actions;
        }

        /** Whether later actions preempt earlier ones on shared resources (path markers). */
        boolean laterWins() {
            return laterWins;
        }
    }

    private static final class ActiveLease {
        private final Action action;
        private final long recency;
        private final StateScheduler scheduler;
        private final Set<Object> reservedResources;
        private final String diagnosticLocation;
        private final Object owner;
        private final boolean narrowable;
        private final Set<Object> liveResources = new LinkedHashSet<>();
        private boolean liveComputed;
        private double startSeconds = Double.NaN;
        private boolean evaluating;
        private boolean released;

        private ActiveLease(
                Action action,
                long recency,
                ActionContext actionContext,
                Map<PathAction, PathRuntime> pathRuntimes,
                RuntimeOverrides overrides,
                java.util.function.ObjDoubleConsumer<ca.frc6390.athena.hardware.device.EncoderDevice> encoderSet,
                Set<Object> reservedResources,
                String diagnosticLocation,
                Object owner,
                boolean narrowable) {
            this.action = action;
            this.recency = recency;
            this.scheduler = new StateScheduler(actionContext, pathRuntimes, overrides, encoderSet);
            this.reservedResources = reservedResources == null ? Set.of() : Set.copyOf(reservedResources);
            this.diagnosticLocation = diagnosticLocation == null || diagnosticLocation.isBlank()
                    ? "requestedAction" : diagnosticLocation;
            this.owner = owner;
            this.narrowable = narrowable;
        }

        private Object owner() {
            return owner;
        }

        /**
         * Recomputes the resources this lease blocks older leases from: those of nodes that are
         * active now or reachable soon. Before the first evaluation the compiled set is used.
         */
        private void refreshReservation(LeaseResources resources) {
            if (!narrowable || resources == null) return;
            liveResources.clear();
            liveComputed = true;
            if (scheduler.complete()) return;
            scheduler.collectLive(action, resources, liveResources);
            liveResources.retainAll(reservedResources);
        }

        private Set<Object> currentReservation() {
            return narrowable && liveComputed ? liveResources : reservedResources;
        }

        private long recency() {
            return recency;
        }

        private Action action() {
            return action;
        }

        private Set<Object> reservedResources() {
            return reservedResources;
        }

        private String diagnosticLocation() {
            return diagnosticLocation;
        }

        private StateScheduler scheduler() {
            return scheduler;
        }

        private double timeInState(double nowSeconds) {
            return Double.isNaN(startSeconds) ? 0.0 : Math.max(0.0, nowSeconds - startSeconds);
        }

        private void shift(double seconds) {
            if (!Double.isNaN(startSeconds)) startSeconds += seconds;
            scheduler.shift(seconds);
        }

        private StateScheduler.Result evaluate(MechanismContext context) {
            if (Double.isNaN(startSeconds)) {
                startSeconds = context.nowSeconds();
            }
            return scheduler.evaluate(action, withTimeInState(context, context.nowSeconds() - startSeconds));
        }
    }

    private static MechanismContext withTimeInState(MechanismContext context, double timeInStateSeconds) {
        return new MechanismContext(
                context.nowSeconds(),
                Math.max(0.0, timeInStateSeconds),
                context.dtSeconds(),
                context.enabled(),
                context.autonomous(),
                context.simulation());
    }

    private static final class StateScheduler {
        private final ActionContext actionContext;
        private final Map<PathAction, PathRuntime> pathRuntimes;
        private final RuntimeOverrides overrides;
        private final java.util.function.ObjDoubleConsumer<ca.frc6390.athena.hardware.device.EncoderDevice> encoderSet;
        private final Result result = new Result();
        private SchedulerNode root;
        private String scheduledActionType = "";
        private int step = -1;
        private boolean complete;

        private StateScheduler(
                ActionContext actionContext,
                Map<PathAction, PathRuntime> pathRuntimes,
                RuntimeOverrides overrides,
                java.util.function.ObjDoubleConsumer<ca.frc6390.athena.hardware.device.EncoderDevice> encoderSet) {
            this.actionContext = Objects.requireNonNull(actionContext, "actionContext");
            this.pathRuntimes = Objects.requireNonNull(pathRuntimes, "pathRuntimes");
            this.overrides = Objects.requireNonNull(overrides, "overrides");
            this.encoderSet = Objects.requireNonNull(encoderSet, "encoderSet");
        }

        void reset() {
            if (root != null) {
                root.reset();
            }
        }

        void interruptLifecycles() {
            if (root != null) {
                root.interruptLifecycles();
            }
        }

        void shift(double seconds) {
            if (root != null) {
                root.shift(seconds);
            }
        }

        Result evaluate(Action action, MechanismContext context) {
            if (root == null || root.action() != action) {
                root = SchedulerNode.lower(action);
            }
            Evaluation evaluation = evaluate(root, context);
            scheduledActionType = typeName(evaluation.output());
            step = root.runtime == null ? -1 : root.runtime.index;
            complete = evaluation.complete();
            return result.set(
                    evaluation.output() == null ? Actions.neutral() : evaluation.output(),
                    evaluation.context());
        }

        String scheduledActionType() {
            return scheduledActionType;
        }

        /**
         * Collects resources this scheduler may drive now or soon: the active node subtree, the
         * next sequence/then stage (so consecutive stages hand a resource over without a gap),
         * both branches of live conditionals, path ownership plus pending and running markers, and
         * every step of a cycle. Stages two or more steps ahead are not reserved.
         */
        void collectLive(Action action, LeaseResources resources, Set<Object> out) {
            if (root == null || root.action() != action) {
                out.addAll(resources.of(action));
                return;
            }
            collectLive(root, resources, out);
        }

        private void collectLive(SchedulerNode schedule, LeaseResources resources, Set<Object> out) {
            Action action = schedule.action();
            Node node = schedule.runtime;
            if (node == null) {
                out.addAll(resources.of(action));
                return;
            }
            if (action instanceof Actions.Sequence sequence) {
                List<Actions.SequenceStep> steps = sequence.steps();
                if (node.terminal) return;
                if (node.index >= steps.size()) {
                    if (sequence.next() != null) {
                        collectChild(schedule.namedChildren.get("next"), sequence.next(), resources, out);
                    }
                    return;
                }
                Action current = steps.get(node.index).action();
                collectChild(schedule.indexedChild(node.index), current, resources, out);
                if (node.index + 1 < steps.size()) {
                    out.addAll(resources.of(steps.get(node.index + 1).action()));
                } else if (sequence.next() != null) {
                    out.addAll(resources.of(sequence.next()));
                }
                return;
            }
            if (action instanceof Actions.Parallel || action instanceof Actions.Race
                    || action instanceof Actions.Deadline) {
                if (node.terminal) return;
                List<Action> children = action instanceof Actions.Parallel parallel ? parallel.Actions()
                        : action instanceof Actions.Race race ? race.Actions()
                        : ((Actions.Deadline) action).Actions();
                for (int index = 0; index < children.size(); index++) {
                    collectChild(schedule.indexedChild(index), children.get(index), resources, out);
                }
                return;
            }
            if (action instanceof Actions.Timeout timeout) {
                if (!node.terminal) {
                    collectChild(schedule.namedChildren.get("timeout"), timeout.action(), resources, out);
                }
                return;
            }
            if (action instanceof Actions.WithinTolerance within) {
                if (!node.terminal) {
                    collectChild(schedule.namedChildren.get("withinTolerance"), within.action(), resources, out);
                }
                return;
            }
            if (action instanceof Actions.Conditional conditional) {
                collectConditional(schedule, node, conditional.action(), conditional.next(), resources, out);
                return;
            }
            if (action instanceof Actions.Then then) {
                collectThen(schedule, node, then.action(), then.next(), resources, out);
                return;
            }
            if (action instanceof Actions.Either && node.entered) {
                if (node.chosen != null) collectChild(schedule.namedChildren.get("either"), node.chosen, resources, out);
                return;
            }
            if (action instanceof PathAction path) {
                if (node.pathComplete) return;
                if (!node.entered || node.pathRuntime == null) {
                    out.addAll(resources.of(path));
                    return;
                }
                out.addAll(resources.pathOwnership(path));
                java.util.Collection<String> pending = node.pathRuntime.pendingMarkers(path);
                for (Map.Entry<String, Action> marker : path.markers().entrySet()) {
                    String name = marker.getKey();
                    if (node.completedMarkers.contains(name)) continue;
                    SchedulerNode running = schedule.namedChildren.get(MARKER_PREFIX + name);
                    if (running != null) {
                        collectChild(running, marker.getValue(), resources, out);
                    } else if (pending == null || pending.contains(name)) {
                        out.addAll(resources.of(marker.getValue()));
                    }
                }
                for (Map.Entry<String, SchedulerNode> child : schedule.namedChildren.entrySet()) {
                    String name = child.getKey();
                    if (name.startsWith(MARKER_PREFIX)
                            && !path.markers().containsKey(name.substring(MARKER_PREFIX.length()))) {
                        collectLive(child.getValue(), resources, out);
                    }
                }
                return;
            }
            if (node.terminal && (action instanceof Actions.Computed
                    || action instanceof Actions.HardwareComputed
                    || action instanceof Actions.VelocityContribution
                    || action instanceof CharacterizationRoutine.Test)) {
                return;
            }
            // Leaves, computed ownership, cycles, and conditional branches reserve their full static
            // shape: a branch may switch on any cycle and a cycle may wrap back to any step.
            out.addAll(resources.of(action));
        }

        private void collectConditional(
                SchedulerNode schedule,
                Node node,
                Action wrapped,
                Action next,
                LeaseResources resources,
                Set<Object> out) {
            if (node.terminal) {
                if (next != null) collectChild(schedule.namedChildren.get("next"), next, resources, out);
                return;
            }
            collectChild(schedule.namedChildren.get("conditional"), wrapped, resources, out);
            if (next != null) collectChild(schedule.namedChildren.get("next"), next, resources, out);
        }

        private void collectThen(
                SchedulerNode schedule,
                Node node,
                Action wrapped,
                Action next,
                LeaseResources resources,
                Set<Object> out) {
            if (!node.terminal) {
                collectChild(schedule.namedChildren.get("thenState"), wrapped, resources, out);
            }
            collectChild(schedule.namedChildren.get("thenNext"), next, resources, out);
        }

        private void collectChild(
                SchedulerNode child,
                Action expected,
                LeaseResources resources,
                Set<Object> out) {
            if (child == null || child.action() != expected) {
                out.addAll(resources.of(expected));
            } else {
                collectLive(child, resources, out);
            }
        }

        int step() {
            return step;
        }

        boolean complete() {
            return complete;
        }

        private Evaluation evaluate(SchedulerNode schedule, MechanismContext context) {
            Objects.requireNonNull(schedule, "schedule");
            Action action = schedule.action();
            Node node = schedule.runtime(context);
            MechanismContext local = withTimeInState(context, context.nowSeconds() - node.startSeconds);

            if (action instanceof Actions.Sequence sequence) {
                return evaluateSequence(sequence, schedule, context, local, node);
            }
            if (action instanceof Actions.Cycle cycle) {
                return evaluateCycle(cycle, schedule, context, node);
            }
            if (action instanceof Actions.Parallel parallel) {
                return evaluateGroup(parallel.Actions(), schedule, context, node, GroupMode.PARALLEL, -1);
            }
            if (action instanceof Actions.Race race) {
                return evaluateGroup(race.Actions(), schedule, context, node, GroupMode.RACE, -1);
            }
            if (action instanceof Actions.Deadline deadline) {
                return evaluateGroup(deadline.Actions(), schedule, context, node, GroupMode.DEADLINE, 0);
            }
            if (action instanceof Actions.Computed computed) {
                if (node.terminal) return schedule.result(null, true, local);
                SchedulerNode childNode = schedule.named("computed", computed.evaluate(local));
                Evaluation child = evaluate(childNode, context);
                node.terminal = child.complete();
                if (child.complete()) childNode.reset();
                return schedule.result(child.output(), child.complete(), child.context());
            }
            if (action instanceof Actions.HardwareComputed computed) {
                if (node.terminal) return schedule.result(null, true, local);
                SchedulerNode childNode = schedule.named("hardwareComputed", computed.evaluate(actionContext));
                Evaluation child = evaluate(childNode, context);
                node.terminal = child.complete();
                if (child.complete()) childNode.reset();
                return schedule.result(child.output(), child.complete(), child.context());
            }
            if (action instanceof Actions.VelocityContribution contribution) {
                if (node.terminal) return schedule.result(null, true, local);
                contribution.channel().set(contribution.sample());
                SchedulerNode childNode = schedule.named("velocityDrive", contribution.driveAction());
                Evaluation child = evaluate(childNode, context);
                if (child.complete()) {
                    contribution.channel().clear();
                    childNode.reset();
                    node.terminal = true;
                }
                return schedule.result(child.output(), child.complete(), child.context());
            }
            if (action instanceof CharacterizationRoutine.Test sysId) {
                if (node.terminal) return schedule.result(null, true, local);
                if (sysId.routine().timedOut(local.timeInStateSeconds())) {
                    sysId.routine().end();
                    node.entered = false;
                    node.terminal = true;
                    return schedule.result(null, true, local);
                }
                node.entered = true;
                return schedule.result(sysId.output(local.timeInStateSeconds()), false, local);
            }
            if (action instanceof Actions.Choice choice) {
                return evaluate(schedule.named("choice", choice.choose(local)), context);
            }
            if (action instanceof Actions.WhenBranch branch) {
                return evaluate(schedule.named("when", branch.choose(local)), context);
            }
            if (action instanceof Actions.Either either) {
                if (!node.entered) {
                    node.entered = true;
                    node.chosen = either.choose(local);
                }
                if (node.chosen == null) return schedule.result(null, true, local);
                return evaluate(schedule.named("either", node.chosen), context);
            }
            if (action instanceof Actions.Timeout timeout) {
                if (node.terminal) return schedule.result(null, true, local);
                if (timeout.expired(local)) {
                    RuntimeEvents.emit(RuntimeEvents.TIMEOUT,
                            "after " + timeout.duration().in(edu.wpi.first.units.Units.Seconds) + " s: " + typeName(timeout.action()));
                    schedule.named("timeout", timeout.action()).reset();
                    node.terminal = true;
                    return schedule.result(null, true, local);
                }
                Evaluation child = evaluate(schedule.named("timeout", timeout.action()), context);
                if (child.complete()) node.terminal = true;
                return schedule.result(child.output(), child.complete(), child.context());
            }
            if (action instanceof Actions.WithinTolerance within) {
                Evaluation child = evaluate(schedule.named("withinTolerance", within.action()), context);
                boolean inside = isWithinTolerance(child.output(), within.tolerance(), within.maxVelocity(), local);
                // The settle timer starts on the first in-window cycle and restarts whenever the
                // measurement leaves the window. With no settle time it is satisfied immediately.
                if (!inside) {
                    node.withinSinceSeconds = Double.NaN;
                } else if (Double.isNaN(node.withinSinceSeconds)) {
                    node.withinSinceSeconds = context.nowSeconds();
                }
                boolean settled = inside
                        && context.nowSeconds() - node.withinSinceSeconds + SETTLE_EPSILON_SECONDS
                                >= within.settleSeconds();
                boolean complete = child.complete() || settled;
                return schedule.result(child.output(), complete, child.context());
            }
            if (action instanceof Actions.Conditional conditional) {
                return evaluateConditional(
                        conditional.action(),
                        conditional.condition(),
                        conditional.next(),
                        schedule,
                        context,
                        local,
                        node);
            }
            if (action instanceof Actions.Then then) {
                return evaluateThen(then.action(), then.next(), schedule, context, node);
            }
            if (action instanceof Actions.RuntimeAction runtimeAction) {
                if (!node.entered) {
                    runtimeAction.action().apply(actionContext);
                    node.entered = true;
                }
                return schedule.result(null, true, local);
            }
            if (action instanceof Actions.EncoderSetPosition setPosition) {
                if (!node.entered) {
                    try {
                        actionContext.encoder(setPosition.encoder()).setPositionRotations(
                                setPosition.encoder().rotationsFromPosition(setPosition.encoder().measure(setPosition.position())));
                        encoderSet.accept(setPosition.encoder(), setPosition.position());
                    } catch (RuntimeException exception) {
                        handleHardwareMutationFailure(
                                setPosition.encoder(), setPosition.encoder().failurePolicy(), exception);
                    }
                    node.entered = true;
                }
                return schedule.result(null, true, local);
            }
            if (action instanceof Actions.ImuYawMutation setYaw) {
                if (!node.entered) {
                    try {
                        setYaw.apply(actionContext);
                    } catch (RuntimeException exception) {
                        Object declaration = imuDeclaration(setYaw);
                        FailurePolicy policy = declaration instanceof ImuDevice imu
                                ? imu.failurePolicy()
                                : FailurePolicy.WARN;
                        handleHardwareMutationFailure(declaration, policy, exception);
                    }
                    node.entered = true;
                }
                return schedule.result(null, true, local);
            }
            if (action instanceof Actions.DoOnce doOnce) {
                if (!node.entered) {
                    doOnce.action().run();
                    node.entered = true;
                }
                return schedule.result(null, true, local);
            }
            if (action instanceof LifecycleAction lifecycle) {
                if (node.terminal) return schedule.result(null, true, local);
                if (!context.enabled()) {
                    // Disabled robots do not run external code. Ending here matches cancellation on disable;
                    // the Action restarts with onStart when the robot is enabled again.
                    if (node.entered) {
                        node.entered = false;
                        lifecycle.end(true);
                    }
                    return schedule.result(null, false, local);
                }
                if (!node.entered) {
                    node.entered = true;
                    lifecycle.initialize();
                }
                lifecycle.execute();
                if (lifecycle.finished()) {
                    node.entered = false;
                    node.terminal = true;
                    lifecycle.end(false);
                    return schedule.result(null, true, local);
                }
                return schedule.result(null, false, local);
            }
            if (action instanceof Actions.WaitSeconds wait) {
                return schedule.result(null, wait.complete(local), local);
            }
            if (action instanceof Actions.WaitUntil wait) {
                return schedule.result(null, wait.complete(local), local);
            }
            if (action instanceof Actions.WaitUntilStalled stall) {
                return schedule.result(null, stalled(stall.signal(), node, local), local);
            }
            if (action instanceof PathAction pathState) {
                if (node.pathComplete) {
                    return schedule.result(null, true, local);
                }
                if (node.pathRuntime == null || node.pathState != pathState) {
                    node.pathState = pathState;
                    node.pathRuntime = pathRuntimes.get(pathState);
                }
                PathRuntime runtime = node.pathRuntime;
                if (runtime == null) {
                    throw new IllegalStateException("No PathRuntime bound for path " + pathState.key());
                }
                if (!node.entered) {
                    runtime.initialize(pathState, local);
                    node.entered = true;
                }
                node.lastPathContext = local;
                runtime.execute(pathState, local);
                if (runtime.isFinished(pathState, local)) {
                    java.util.Collection<String> pending = runtime.pendingMarkers(pathState);
                    if (pending != null && !pending.isEmpty()) {
                        RuntimeEvents.emit(RuntimeEvents.PATH_MARKERS_PENDING,
                                pathState.key() + " pending markers " + pending);
                    }
                    runtime.end(pathState, local, false);
                    node.entered = false;
                    node.pathComplete = true;
                    return schedule.result(null, true, local);
                }
                node.groupOutputs.clear();
                // Markers run before the drive output is requested so instant markers such as an
                // odometry reset are applied before the follower computes this cycle's sample.
                Map<String, Action> activeMarkers = Objects.requireNonNull(
                        runtime.activeMarkers(pathState, local), "active path markers");
                node.completedMarkers.retainAll(activeMarkers.keySet());
                schedule.retainMarkerChildren(activeMarkers.keySet());
                List<Action> markerOutputs = node.markerOutputs;
                markerOutputs.clear();
                for (Map.Entry<String, Action> entry : activeMarkers.entrySet()) {
                    String name = entry.getKey();
                    if (node.completedMarkers.contains(name)) continue;
                    String childName = MARKER_PREFIX + name;
                    Evaluation markerOutput = evaluate(schedule.named(childName, entry.getValue()), context);
                    if (markerOutput.complete()) {
                        // Finite marker actions stop once complete; continuous ones run until path end.
                        node.completedMarkers.add(name);
                        schedule.removeNamed(childName);
                        runtime.markerCompleted(pathState, name);
                        continue;
                    }
                    if (markerOutput.output() != null) markerOutputs.add(markerOutput.output());
                }
                Action pathOutput = runtime.output(pathState, local);
                if (pathOutput != null) {
                    Evaluation drive = evaluate(schedule.named("pathOutput", pathOutput), context);
                    if (drive.output() != null) node.groupOutputs.add(drive.output());
                } else {
                    schedule.removeNamed("pathOutput");
                }
                // Later-triggered markers follow earlier ones; the resolver lets later outputs win a
                // shared motor or sink instead of failing the cycle with an output conflict.
                node.groupOutputs.addAll(markerOutputs);
                if (node.groupOutputs.isEmpty()) return schedule.result(null, false, local);
                return schedule.result(
                        node.groupOutputs.size() == 1 ? node.groupOutputs.get(0) : node.precedenceOutputs,
                        false,
                        local);
            }
            return schedule.result(action, false, local);
        }

        private Evaluation evaluateSequence(
                Actions.Sequence sequence,
                SchedulerNode schedule,
                MechanismContext context,
                MechanismContext local,
                Node node) {
            if (node.terminal) {
                return schedule.result(null, true, local);
            }
            if (Actions.hasElapsed(local.timeInStateSeconds(), sequence.timeoutSeconds())) {
                // Like timeout(...) on any other action, a sequence timeout ends the whole sequence,
                // including stages appended with then(...).
                schedule.resetChildren();
                node.terminal = true;
                return schedule.result(null, true, local);
            }
            List<Actions.SequenceStep> steps = sequence.steps();
            while (node.index < steps.size()) {
                Actions.SequenceStep step = steps.get(node.index);
                SchedulerNode stepNode = schedule.indexed(node.index, step.action());
                MechanismContext stepContext = stepNode.localContext(context);
                Evaluation child = evaluate(stepNode, context);
                if (child.complete() || step.complete().test(stepContext)) {
                    stepNode.reset();
                    node.index++;
                    continue;
                }
                return child;
            }
            if (sequence.next() != null) {
                return evaluate(schedule.named("next", sequence.next()), context);
            }
            return schedule.result(null, true, local);
        }

        private Evaluation evaluateCycle(
                Actions.Cycle cycle,
                SchedulerNode schedule,
                MechanismContext context,
                Node node) {
            List<Actions.CycleStep> steps = cycle.steps();
            if (steps.isEmpty()) {
                return schedule.result(null, false, context);
            }
            if (node.index >= steps.size()) {
                node.index = 0;
            }
            for (int attempts = 0; attempts < steps.size(); attempts++) {
                Actions.CycleStep step = steps.get(node.index);
                SchedulerNode stepNode = schedule.indexed(node.index, step.action());
                MechanismContext stepContext = stepNode.localContext(context);
                Evaluation child = evaluate(stepNode, context);
                if (child.complete() || step.advance().test(stepContext)) {
                    stepNode.reset();
                    node.index = (node.index + 1) % steps.size();
                    continue;
                }
                return child;
            }
            return schedule.result(null, false, context);
        }

        private Evaluation evaluateGroup(
                List<Action> actions,
                SchedulerNode schedule,
                MechanismContext context,
                Node node,
                GroupMode mode,
                int deadlineIndex) {
            if (node.terminal) {
                return schedule.result(null, true, context);
            }
            List<Action> outputs = node.groupOutputs;
            outputs.clear();
            boolean anyComplete = false;
            boolean allComplete = !actions.isEmpty();
            boolean deadlineComplete = false;
            if (mode == GroupMode.PARALLEL
                    && (node.latchedComplete == null || node.latchedComplete.length != actions.size())) {
                node.latchedComplete = new boolean[actions.size()];
            }
            for (int i = 0; i < actions.size(); i++) {
                Evaluation child = evaluate(schedule.indexed(i, actions.get(i)), context);
                boolean childComplete = child.complete();
                if (mode == GroupMode.PARALLEL) {
                    // A child that reached completion once (for example untilWithin on a shooter
                    // spin-up) stays complete for the group, but keeps contributing its output so
                    // it holds until the whole group finishes.
                    childComplete |= node.latchedComplete[i];
                    node.latchedComplete[i] = childComplete;
                }
                anyComplete |= childComplete;
                allComplete &= childComplete;
                if (mode == GroupMode.DEADLINE && i == deadlineIndex && child.complete()) {
                    deadlineComplete = true;
                }
                if (child.output() != null) {
                    outputs.add(child.output());
                }
            }
            if ((mode == GroupMode.RACE && anyComplete)
                    || (mode == GroupMode.DEADLINE && deadlineComplete)) {
                schedule.resetChildren();
                node.terminal = true;
                return schedule.result(null, true, context);
            }
            if (mode == GroupMode.PARALLEL && allComplete) {
                schedule.resetChildren();
                node.terminal = true;
                return schedule.result(null, true, context);
            }
            return schedule.result(groupOutput(node), false, context);
        }

        private Evaluation evaluateConditional(
                Action wrapped,
                ActionCondition condition,
                Action next,
                SchedulerNode schedule,
                MechanismContext context,
                MechanismContext local,
                Node node) {
            if (node.terminal || condition.test(local)) {
                if (!node.terminal) {
                    schedule.named("conditional", wrapped).reset();
                    node.terminal = true;
                }
                return next == null ? schedule.result(null, true, local) : evaluate(schedule.named("next", next), context);
            }
            SchedulerNode childNode = schedule.named("conditional", wrapped);
            Evaluation child = evaluate(childNode, context);
            if (child.complete()) {
                // Latch: once the wrapped action finishes on its own, later cycles must not fall
                // back into it (for example a waitUntil that turns false again or an untilWithin
                // that drifts out of tolerance) and abandon the next stage.
                node.terminal = true;
                if (next == null) return child;
                childNode.reset();
                return evaluate(schedule.named("next", next), context);
            }
            return child;
        }

        /** Debounces a stall detector against this node's own start, so every run starts fresh. */
        private boolean stalled(
                ca.frc6390.athena.hardware.signal.MotorStallSignal signal,
                Node node,
                MechanismContext context) {
            if (node.terminal) return true;
            boolean raw = RuntimeHardwareAccess.call(actionContext, signal::instantaneousActive);
            if (!raw) {
                node.stallSinceSeconds = Double.NaN;
                return false;
            }
            if (Double.isNaN(node.stallSinceSeconds)) node.stallSinceSeconds = context.nowSeconds();
            if (Actions.hasElapsed(context.nowSeconds() - node.stallSinceSeconds, signal.raw().durationSeconds())) {
                node.terminal = true;
            }
            return node.terminal;
        }

        private Evaluation evaluateThen(
                Action wrapped,
                Action next,
                SchedulerNode schedule,
                MechanismContext context,
                Node node) {
            if (node.terminal) return evaluate(schedule.named("thenNext", next), context);
            SchedulerNode childNode = schedule.named("thenState", wrapped);
            Evaluation child = evaluate(childNode, context);
            if (child.complete()) {
                childNode.reset();
                node.terminal = true;
                return evaluate(schedule.named("thenNext", next), context);
            }
            return child;
        }

        private boolean isWithinTolerance(
                Action output, double tolerance, double maxVelocity, MechanismContext context) {
            ControlTarget target = controlTarget(output);
            if (target == null || !Double.isFinite(target.value())) {
                return false;
            }
            ControlBinding<?> control = target.control();
            double measurement = RuntimeHardwareAccess.call(actionContext, () -> switch (target.mode()) {
                case POSITION -> feedbackPosition(control);
                case VELOCITY -> feedbackVelocity(control);
            });
            if (!Double.isFinite(measurement)) {
                return false;
            }
            double goal = constrainedTarget(target, measurement, context);
            if (!Double.isFinite(goal)) {
                return false;
            }
            double error = goal - measurement;
            if (target.mode() == ControlMode.POSITION && control.continuousRange() != null) {
                double span = control.continuousRange().raw().maximumInUnit() - control.continuousRange().raw().minimumInUnit();
                error = Math.IEEEremainder(error, span);
            }
            if (Math.abs(error) > tolerance) {
                return false;
            }
            if (Double.isFinite(maxVelocity)) {
                double velocity = RuntimeHardwareAccess.call(actionContext, () -> feedbackVelocity(control));
                return Math.abs(velocity) <= maxVelocity;
            }
            return true;
        }

        /**
         * Returns the target the control will actually drive to: after range constraints, the
         * motion planner, and dashboard-tuned limits. Comparing against the raw request would never
         * complete for out-of-range targets. Returns NaN when the request is rejected.
         */
        private double constrainedTarget(ControlTarget target, double measurement, MechanismContext context) {
            ControlBinding<?> control = target.control();
            RuntimeOverrides.ControlTuning tuning = overrides.tuning(control);
            if (target.mode() == ControlMode.POSITION) {
                double requested = target.value();
                if (overrides.hasTuning(control)) {
                    requested = Math.max(tuning.minimumPosition(), Math.min(tuning.maximumPosition(), requested));
                }
                ca.frc6390.athena.mechanism.constraint.ConstraintContext<Double> constraintContext =
                        new ca.frc6390.athena.mechanism.constraint.ConstraintContext<>(measurement, requested, context);
                ca.frc6390.athena.mechanism.constraint.ConstraintResult<Double> result = control.planner() == null
                        ? ca.frc6390.athena.mechanism.constraint.Constraints.evaluate(
                                control.constraints(), constraintContext)
                        : control.planner().plan(constraintContext, control.constraints());
                if (!result.accepted()) return Double.NaN;
                return Math.max(tuning.minimumPosition(), Math.min(tuning.maximumPosition(), result.value()));
            }
            double requested = Math.max(-tuning.maxVelocity(), Math.min(tuning.maxVelocity(), target.value()));
            ca.frc6390.athena.mechanism.constraint.ConstraintResult<Double> result =
                    ca.frc6390.athena.mechanism.constraint.Constraints.evaluate(
                            control.constraints(),
                            new ca.frc6390.athena.mechanism.constraint.ConstraintContext<>(
                                    measurement, requested, context));
            return result.accepted() ? result.value() : Double.NaN;
        }

        private double feedbackPosition(ControlBinding<?> control) {
            if (control.feedback() != null) {
                return RuntimeHardwareAccess.call(actionContext, control.feedback().position()::position);
            }
            return actionContext.motor(control.output()).integratedPositionRotations();
        }

        private double feedbackVelocity(ControlBinding<?> control) {
            if (control.feedback() != null) {
                return RuntimeHardwareAccess.call(actionContext, control.feedback().velocity()::velocity);
            }
            return actionContext.motor(control.output()).integratedVelocityRotationsPerSecond();
        }

        private void handleHardwareMutationFailure(
                Object declaration,
                FailurePolicy policy,
                RuntimeException exception) {
            if (policy == FailurePolicy.PANIC) throw FaultIsolation.panic(exception);
            actionContext.hardwareFailure(declaration, exception);
        }

        private static Object imuDeclaration(Actions.ImuYawMutation mutation) {
            if (mutation.imu() instanceof ImuDevice imu) return imu;
            for (Object dependency : mutation.imu().dependencies()) {
                if (dependency instanceof ImuDevice imu) return imu;
            }
            return mutation.imu();
        }

        private static ControlTarget controlTarget(Action action) {
            if (action instanceof Actions.ControlPosition position) {
                return new ControlTarget(position.control(), ControlMode.POSITION, position.position());
            }
            if (action instanceof Actions.DynamicControlPosition position) {
                return new ControlTarget(
                        position.control(), ControlMode.POSITION, position.positionSupplier().getAsDouble());
            }
            if (action instanceof Actions.ControlVelocity velocity) {
                return new ControlTarget(velocity.control(), ControlMode.VELOCITY, velocity.velocity());
            }
            if (action instanceof Actions.DynamicControlVelocity velocity) {
                return new ControlTarget(
                        velocity.control(), ControlMode.VELOCITY, velocity.velocitySupplier().getAsDouble());
            }
            if (action instanceof InterpolatedControlAction interpolated) {
                return new ControlTarget(
                        interpolated.control(),
                        interpolated.control().mode(),
                        interpolated.runtimeTarget());
            }
            return null;
        }

        private static Action groupOutput(Node node) {
            List<Action> actions = node.groupOutputs;
            if (actions.isEmpty()) {
                return null;
            }
            if (actions.size() == 1) {
                return actions.get(0);
            }
            return node.scheduledOutputs;
        }

        private enum GroupMode {
            PARALLEL,
            RACE,
            DEADLINE
        }

        private record ControlTarget(ControlBinding<?> control, ControlMode mode, double value) {
        }

        private static final class Evaluation {
            private Action output;
            private boolean complete;
            private MechanismContext context;

            private Evaluation set(Action output, boolean complete, MechanismContext context) {
                this.output = output;
                this.complete = complete;
                this.context = context;
                return this;
            }

            private Action output() {
                return output;
            }

            private boolean complete() {
                return complete;
            }

            private MechanismContext context() {
                return context;
            }
        }

        private static final class Result {
            private Action action;
            private MechanismContext context;

            private Result set(Action action, MechanismContext context) {
                this.action = action;
                this.context = context;
                return this;
            }

            private Action action() {
                return action;
            }

            private MechanismContext context() {
                return context;
            }
        }

        private static final class Node {
            private double startSeconds;
            private double stallSinceSeconds = Double.NaN;
            private Action chosen;
            private boolean entered;
            private int index;
            private PathAction pathState;
            private PathRuntime pathRuntime;
            private MechanismContext lastPathContext;
            private boolean pathComplete;
            private boolean terminal;
            private double withinSinceSeconds = Double.NaN;
            private boolean[] latchedComplete;
            private final List<Action> groupOutputs = new ArrayList<>();
            private final ScheduledOutputs scheduledOutputs = new ScheduledOutputs(groupOutputs);
            private final ScheduledOutputs precedenceOutputs = new ScheduledOutputs(groupOutputs, true);
            private final List<Action> markerOutputs = new ArrayList<>();
            private final Set<String> completedMarkers = new java.util.HashSet<>();

            private Node(double startSeconds) {
                this.startSeconds = startSeconds;
            }
        }

        private static final class SchedulerNode {
            private final Action action;
            private final List<SchedulerNode> indexedChildren = new ArrayList<>();
            private final Map<String, SchedulerNode> namedChildren = new HashMap<>();
            private final Evaluation evaluation = new Evaluation();
            private Node runtime;

            private SchedulerNode(Action action) {
                this.action = Objects.requireNonNull(action, "action");
                Actions.validate(action);
                lowerKnownChildren();
            }

            private static SchedulerNode lower(Action action) {
                return new SchedulerNode(action);
            }

            private Action action() {
                return action;
            }

            private Node runtime(MechanismContext context) {
                if (runtime == null) {
                    runtime = new Node(context.nowSeconds());
                }
                return runtime;
            }

            private MechanismContext localContext(MechanismContext context) {
                Node node = runtime(context);
                return withTimeInState(context, context.nowSeconds() - node.startSeconds);
            }

            private SchedulerNode indexedChild(int index) {
                return index < indexedChildren.size() ? indexedChildren.get(index) : null;
            }

            private SchedulerNode indexed(int index, Action childState) {
                while (indexedChildren.size() <= index) {
                    indexedChildren.add(null);
                }
                SchedulerNode child = indexedChildren.get(index);
                if (child == null || child.action() != childState) {
                    if (child != null) child.reset();
                    child = lower(childState);
                    indexedChildren.set(index, child);
                }
                return child;
            }

            private SchedulerNode named(String name, Action childState) {
                SchedulerNode child = namedChildren.get(name);
                if (child == null || child.action() != childState) {
                    if (child != null) child.reset();
                    child = lower(childState);
                    namedChildren.put(name, child);
                }
                return child;
            }

            private void reset() {
                if (runtime != null && runtime.entered && action instanceof CharacterizationRoutine.Test sysId) {
                    sysId.routine().end();
                }
                if (action instanceof Actions.VelocityContribution contribution) {
                    contribution.channel().clear();
                }
                if (runtime != null && runtime.entered && action instanceof LifecycleAction lifecycle) {
                    // Cancellation, replacement, timeout, or a lost race before the Action finished.
                    runtime.entered = false;
                    lifecycle.end(true);
                }
                if (runtime != null && runtime.entered && runtime.pathRuntime != null && runtime.pathState != null) {
                    runtime.pathRuntime.end(
                            runtime.pathState,
                            runtime.lastPathContext == null ? MechanismContext.empty() : runtime.lastPathContext,
                            true);
                }
                runtime = null;
                for (SchedulerNode child : indexedChildren) {
                    if (child != null) {
                        child.reset();
                    }
                }
                namedChildren.values().forEach(SchedulerNode::reset);
            }

            private void interruptLifecycles() {
                if (runtime != null && runtime.entered && action instanceof LifecycleAction lifecycle) {
                    runtime.entered = false;
                    lifecycle.end(true);
                }
                for (SchedulerNode child : indexedChildren) {
                    if (child != null) child.interruptLifecycles();
                }
                namedChildren.values().forEach(SchedulerNode::interruptLifecycles);
            }

            private void shift(double seconds) {
                if (runtime != null) {
                    runtime.startSeconds += seconds;
                    runtime.stallSinceSeconds = Double.NaN;
                }
                for (SchedulerNode child : indexedChildren) {
                    if (child != null) child.shift(seconds);
                }
                namedChildren.values().forEach(child -> child.shift(seconds));
            }

            private void resetChildren() {
                for (SchedulerNode child : indexedChildren) {
                    if (child != null) child.reset();
                }
                namedChildren.values().forEach(SchedulerNode::reset);
            }

            private void removeNamed(String name) {
                SchedulerNode removed = namedChildren.remove(name);
                if (removed != null) removed.reset();
            }

            private void retainMarkerChildren(Set<String> activeMarkers) {
                if (namedChildren.isEmpty()) return;
                List<String> inactive = null;
                for (String name : namedChildren.keySet()) {
                    if (name.startsWith(MARKER_PREFIX)
                            && !activeMarkers.contains(name.substring(MARKER_PREFIX.length()))) {
                        if (inactive == null) inactive = new ArrayList<>();
                        inactive.add(name);
                    }
                }
                if (inactive != null) inactive.forEach(this::removeNamed);
            }

            private Evaluation result(Action output, boolean complete, MechanismContext context) {
                return evaluation.set(output, complete, context);
            }

            private void lowerKnownChildren() {
                if (action instanceof Actions.Sequence sequence) {
                    int index = 0;
                    for (Actions.SequenceStep step : sequence.steps()) {
                        indexed(index++, step.action());
                    }
                    if (sequence.next() != null) {
                        named("next", sequence.next());
                    }
                } else if (action instanceof Actions.Cycle cycle) {
                    int index = 0;
                    for (Actions.CycleStep step : cycle.steps()) {
                        indexed(index++, step.action());
                    }
                } else if (action instanceof Actions.Parallel parallel) {
                    lowerIndexed(parallel.Actions());
                } else if (action instanceof Actions.Race race) {
                    lowerIndexed(race.Actions());
                } else if (action instanceof Actions.Deadline deadline) {
                    lowerIndexed(deadline.Actions());
                } else if (action instanceof Actions.Timeout timeout) {
                    named("timeout", timeout.action());
                } else if (action instanceof Actions.WithinTolerance within) {
                    named("withinTolerance", within.action());
                } else if (action instanceof Actions.VelocityContribution contribution) {
                    named("velocityDrive", contribution.driveAction());
                } else if (action instanceof Actions.Conditional conditional) {
                    lowerConditional(conditional.action(), conditional.next());
                } else if (action instanceof Actions.Then then) {
                    lowerThen(then.action(), then.next());
                }
            }

            private void lowerIndexed(List<Action> actions) {
                for (int i = 0; i < actions.size(); i++) {
                    indexed(i, actions.get(i));
                }
            }

            private void lowerConditional(Action wrapped, Action next) {
                named("conditional", wrapped);
                if (next != null) {
                    named("next", next);
                }
            }

            private void lowerThen(Action wrapped, Action next) {
                named("thenState", wrapped);
                named("thenNext", next);
            }
        }
    }
}
