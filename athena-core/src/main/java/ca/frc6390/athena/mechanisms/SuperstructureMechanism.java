package ca.frc6390.athena.mechanisms;

import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.RobotCoreHooks;
import ca.frc6390.athena.core.diagnostics.BoundedEventLog;
import ca.frc6390.athena.core.diagnostics.DiagnosticsChannel;
import ca.frc6390.athena.core.input.TypedInputResolver;
import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.FlywheelMechanism.StatefulFlywheelMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.core.RobotNetworkTables;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;
import java.util.LinkedHashMap;

/**
 * Composite mechanism that coordinates multiple stateful mechanisms using a superstate enum.
 * Constraints reference child mechanisms via {@link SuperstructureContext#getMechanisms()} using
 * the same mapper supplied to the config.
 */
public class SuperstructureMechanism<S extends Enum<S> & SetpointProvider<SP>, SP> extends SubsystemBase implements RobotSendableSystem, RegisterableMechanism {
    private RobotCore<?> robotCore;
    private SuperstructureConfig<?, ?> sourceConfig;

    static final class Child<SP, E extends Enum<E> & SetpointProvider<?>> {
        final Mechanism mechanism;
        final SuperstructureMechanism<?, ?> superstructure;
        final StateMachine<?, ?> stateMachine;
        final Function<SP, E> mapper;
        final Class<?> stateType;

        @SuppressWarnings("unchecked")
        <M extends Enum<M> & SetpointProvider<Double>> Child(Mechanism mechanism, Function<SP, M> mapper, Class<M> stateType) {
            this.mechanism = mechanism;
            this.superstructure = null;
            StatefulLike<M> stateful = (StatefulLike<M>) mechanism;
            this.stateMachine = castMachine(stateful.getStateMachine());
            this.mapper = castMapper(mapper);
            this.stateType = stateType;
        }

        <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> Child(SuperstructureMechanism<CS, CSP> superstructure, Function<SP, CS> mapper, Class<CS> stateType) {
            this.mechanism = null;
            this.superstructure = superstructure;
            this.stateMachine = castMachine(superstructure.getStateMachine());
            this.mapper = castMapper(mapper);
            this.stateType = stateType;
        }

        @SuppressWarnings("unchecked")
        private <X> Function<SP, E> castMapper(Function<SP, X> mapper) {
            return (Function<SP, E>) mapper;
        }

        private StateMachine<?, ?> castMachine(StateMachine<?, ?> machine) {
            return machine;
        }
    }

    private final StateMachine<SP, S> stateMachine;
    private final List<Child<SP, ?>> children;
    private final Map<S, SuperstructureConfig.Constraint<S, SP>> constraints;
    private final List<SuperstructureConfig.Attachment<SP, ?>> attachments;
    private final Map<String, java.util.function.BooleanSupplier> inputs;
    private final Map<String, DoubleSupplier> doubleInputs;
    private final Map<String, IntSupplier> intInputs;
    private final Map<String, Supplier<String>> stringInputs;
    private final Map<String, Supplier<Pose2d>> pose2dInputs;
    private final Map<String, Supplier<Pose3d>> pose3dInputs;
    private final Map<String, Supplier<?>> objectInputs;
    private final TypedInputResolver inputResolver;
    private final Map<S, List<SuperstructureConfig.Binding<SP>>> enterBindings;
    private final List<SuperstructureConfig.TransitionBinding<SP, S>> transitionBindings;
    private final Map<S, List<SuperstructureConfig.Binding<SP>>> bindings;
    private final List<SuperstructureConfig.Binding<SP>> alwaysBindings;
    private final List<SuperstructureConfig.Binding<SP>> periodicBindings;
    private final Map<S, List<SuperstructureConfig.Binding<SP>>> exitBindings;
    private final List<SuperstructureConfig.Binding<SP>> exitAlwaysBindings;
    private final SuperstructureContextImpl context;
    private S prevState;
    private final BoundedEventLog<DiagnosticsChannel.Event> diagnosticsLog =
            new BoundedEventLog<>(DIAGNOSTIC_LOG_CAPACITY);
    private final DiagnosticsView diagnosticsView = new DiagnosticsView();
    private static final int DIAGNOSTIC_LOG_CAPACITY = 256;

    SuperstructureMechanism(S initialState,
                            double stateMachineDelaySeconds,
                            List<Child<SP, ?>> children,
                            Map<S, SuperstructureConfig.Constraint<S, SP>> constraints,
                            List<SuperstructureConfig.Attachment<SP, ?>> attachments,
                            Map<String, java.util.function.BooleanSupplier> inputs,
                            Map<String, DoubleSupplier> doubleInputs,
                            Map<String, IntSupplier> intInputs,
                            Map<String, Supplier<String>> stringInputs,
                            Map<String, Supplier<Pose2d>> pose2dInputs,
                            Map<String, Supplier<Pose3d>> pose3dInputs,
                            Map<String, Supplier<?>> objectInputs,
                            Map<S, List<SuperstructureConfig.Binding<SP>>> enterBindings,
                            List<SuperstructureConfig.TransitionBinding<SP, S>> transitionBindings,
                            Map<S, List<SuperstructureConfig.Binding<SP>>> bindings,
                            List<SuperstructureConfig.Binding<SP>> alwaysBindings,
                            List<SuperstructureConfig.Binding<SP>> periodicBindings,
                            Map<S, List<SuperstructureConfig.Binding<SP>>> exitBindings,
                            List<SuperstructureConfig.Binding<SP>> exitAlwaysBindings) {
        this.children = children;
        this.constraints = constraints;
        this.attachments = attachments;
        this.inputs = inputs;
        this.doubleInputs = doubleInputs;
        this.intInputs = intInputs;
        this.stringInputs = stringInputs;
        this.pose2dInputs = pose2dInputs;
        this.pose3dInputs = pose3dInputs;
        this.objectInputs = objectInputs;
        this.inputResolver = new TypedInputResolver(
                "Superstructure",
                TypedInputResolver.ValueMode.LENIENT,
                TypedInputResolver.NO_MUTABLES,
                inputs,
                doubleInputs,
                intInputs,
                stringInputs,
                pose2dInputs,
                pose3dInputs,
                objectInputs);
        this.enterBindings = enterBindings;
        this.transitionBindings = transitionBindings;
        this.bindings = bindings;
        this.alwaysBindings = alwaysBindings;
        this.periodicBindings = periodicBindings;
        this.exitBindings = exitBindings;
        this.exitAlwaysBindings = exitAlwaysBindings;
        this.stateMachine = new StateMachine<>(initialState, this::childrenAtGoals);
        this.stateMachine.setAtStateDelay(stateMachineDelaySeconds);
        this.context = new SuperstructureContextImpl();
        applySetpoints(initialState.getSetpoint());
        this.prevState = initialState;
    }

    /**
     * State machine driving the composite.
     */
    public StateMachine<SP, S> getStateMachine() {
        return stateMachine;
    }

    public DiagnosticsView diagnostics() {
        return diagnosticsView;
    }

    private String diagnosticsSystemKey() {
        String name = getName();
        if (name == null || name.isBlank()) {
            name = getClass().getSimpleName();
        }
        return "superstructures/" + name;
    }

    /**
     * Queues a superstate, applying its constraint if present.
     */
    public void queueState(S state) {
        if (state != null) {
            appendDiagnosticLog("INFO", "state", "queued state '" + state.name() + "'");
        }
        SuperstructureConfig.Constraint<S, SP> constraint = constraints.get(state);
        if (constraint == null) {
            stateMachine.queueState(state);
            return;
        }
        java.util.function.BooleanSupplier guard = () -> constraint.guard.test(context);
        if (!constraint.transitionStates.isEmpty() && !guard.getAsBoolean()) {
            for (S transition : constraint.transitionStates) {
                stateMachine.queueState(transition);
            }
        }
        stateMachine.queueState(state, guard);
    }

    /**
     * Returns true when every child mechanism reports its goal state achieved.
     */
    public boolean childrenAtGoals() {
        for (Child<SP, ?> child : children) {
            if (child.mechanism instanceof StatefulLike<?> stateful) {
                if (!stateful.getStateMachine().atGoalState()) {
                    return false;
                }
            } else if (child.superstructure != null) {
                if (!child.superstructure.childrenAtGoals()) {
                    return false;
                }
            }
            else {
                return false;
            }
        }
        return true;
    }

    @Override
    public void periodic() {
        applyPeriodicBindings();
        stateMachine.update();
        applyAttachments();
        S current = stateMachine.getGoalState();
        boolean changed = !Objects.equals(current, prevState);
        S from = prevState;
        if (changed) {
            applyExitBindings(from);
            applySetpoints(stateMachine.getGoalStateSetpoint());
        }
        if (changed) {
            applyTransitionBindings(from, current);
            applyEnterBindings(current);
            String fromName = from != null ? from.name() : "<none>";
            String toName = current != null ? current.name() : "<none>";
            appendDiagnosticLog("INFO", "transition", fromName + " -> " + toName);
            prevState = current;
        }
        applyBindings(current);
    }

    private void applyEnterBindings(S state) {
        if (state == null) {
            return;
        }
        List<SuperstructureConfig.Binding<SP>> stateBindings = enterBindings.get(state);
        if (stateBindings == null) {
            return;
        }
        for (SuperstructureConfig.Binding<SP> binding : stateBindings) {
            binding.apply(context);
        }
    }

    private void applyTransitionBindings(S from, S to) {
        if (from == null || to == null || transitionBindings == null) {
            return;
        }
        for (SuperstructureConfig.TransitionBinding<SP, S> binding : transitionBindings) {
            if (binding == null || binding.hook() == null) {
                continue;
            }
            if (!Objects.equals(binding.from(), from) || !Objects.equals(binding.to(), to)) {
                continue;
            }
            binding.hook().apply(context, from, to);
        }
    }

    private void applyPeriodicBindings() {
        for (SuperstructureConfig.Binding<SP> binding : periodicBindings) {
            binding.apply(context);
        }
    }

    private void applyAttachments() {
        for (SuperstructureConfig.Attachment<SP, ?> attachment : attachments) {
            setAttachmentPose(attachment);
        }
    }

    private void applyBindings(S state) {
        for (SuperstructureConfig.Binding<SP> binding : alwaysBindings) {
            binding.apply(context);
        }
        List<SuperstructureConfig.Binding<SP>> stateBindings = bindings.get(state);
        if (stateBindings == null) {
            return;
        }
        for (SuperstructureConfig.Binding<SP> binding : stateBindings) {
            binding.apply(context);
        }
    }

    private void applyExitBindings(S state) {
        if (state == null) {
            return;
        }
        context.setOverrideSetpoint(state.getSetpoint());
        for (SuperstructureConfig.Binding<SP> binding : exitAlwaysBindings) {
            binding.apply(context);
        }
        List<SuperstructureConfig.Binding<SP>> stateBindings = exitBindings.get(state);
        if (stateBindings != null) {
            for (SuperstructureConfig.Binding<SP> binding : stateBindings) {
                binding.apply(context);
            }
        }
        context.clearOverrideSetpoint();
    }

    @SuppressWarnings("unchecked")
    private <E extends Enum<E> & SetpointProvider<Double>> void setAttachmentPose(SuperstructureConfig.Attachment<SP, E> attachment) {
        try {
            Mechanism mech = null;
            if (attachment.resolver != null) {
                mech = attachment.resolver.apply(context);
            } else if (attachment.childMapper != null) {
                StatefulLike<E> child = context.mechanism(attachment.childMapper);
                if (child instanceof Mechanism resolved) {
                    mech = resolved;
                }
            }
            if (mech == null) {
                return;
            }
            var pose = attachment.poseSupplier.apply(context);
            mech.setVisualizationRootOverride(pose);
        } catch (Exception ignored) {
            // If the mapper does not resolve, skip silently.
        }
    }

    private void applySetpoints(SP setpoint) {
        for (Child<SP, ?> child : children) {
            queueChild(child, setpoint);
        }
    }

    @SuppressWarnings("unchecked")
    private <E extends Enum<E> & SetpointProvider<?>> void queueChild(Child<SP, E> child, SP setpoint) {
        E childState = child.mapper.apply(setpoint);
        if (childState != null) {
            StateMachine<?, E> stateMachine = (StateMachine<?, E>) child.stateMachine;
            stateMachine.queueState(childState);
        }
    }

    @Override
    public RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return node;
        }
        RobotNetworkTables nt = node.robot();
        if (!nt.isPublishingEnabled()) {
            return node;
        }

        RobotNetworkTables.Node meta = node.child("Meta");
        String name = getName();
        meta.putString("name", name != null ? name : "");
        meta.putString("type", "Superstructure");
        meta.putString("owner", "");
        meta.putString("hint",
                "Enable " + node.path() + "/NetworkTableConfig/Details and section flags to publish more topics.");

        RobotNetworkTables.MechanismToggles toggles = nt.mechanismConfig(node);
        boolean details = toggles.detailsEnabled();
        if (!details) {
            return node;
        }

        if (toggles.controlEnabled()) {
            stateMachine.networkTables(node.child("Control").child("StateMachine"));
            RobotNetworkTables.Node status = node.child("Control").child("Status");
            S state = stateMachine.getGoalState();
            status.putString("state", state != null ? state.name() : "");
        }

        if (toggles.inputsEnabled()) {
            RobotNetworkTables.Node inputsNode = node.child("Inputs");
            RobotNetworkTables.Node bools = inputsNode.child("Bool");
            for (Map.Entry<String, java.util.function.BooleanSupplier> e : inputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                bools.putBoolean(e.getKey(), e.getValue().getAsBoolean());
            }
            RobotNetworkTables.Node dbls = inputsNode.child("Double");
            for (Map.Entry<String, DoubleSupplier> e : doubleInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                dbls.putDouble(e.getKey(), e.getValue().getAsDouble());
            }
            RobotNetworkTables.Node ints = inputsNode.child("Int");
            for (Map.Entry<String, IntSupplier> e : intInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                ints.putDouble(e.getKey(), e.getValue().getAsInt());
            }
            RobotNetworkTables.Node strs = inputsNode.child("String");
            for (Map.Entry<String, Supplier<String>> e : stringInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                strs.putString(e.getKey(), e.getValue().get());
            }
            RobotNetworkTables.Node poses2d = inputsNode.child("Pose2d");
            for (Map.Entry<String, Supplier<Pose2d>> e : pose2dInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                Pose2d pose = e.getValue().get();
                if (pose == null) {
                    continue;
                }
                RobotNetworkTables.Node p = poses2d.child(e.getKey());
                p.putDouble("x", pose.getX());
                p.putDouble("y", pose.getY());
                p.putDouble("deg", pose.getRotation().getDegrees());
            }
            RobotNetworkTables.Node poses3d = inputsNode.child("Pose3d");
            for (Map.Entry<String, Supplier<Pose3d>> e : pose3dInputs.entrySet()) {
                if (e == null || e.getKey() == null || e.getValue() == null) {
                    continue;
                }
                Pose3d pose = e.getValue().get();
                if (pose == null) {
                    continue;
                }
                RobotNetworkTables.Node p = poses3d.child(e.getKey());
                p.putDouble("x", pose.getX());
                p.putDouble("y", pose.getY());
                p.putDouble("z", pose.getZ());
                p.putDouble("rxDeg", pose.getRotation().getX() * 180.0 / Math.PI);
                p.putDouble("ryDeg", pose.getRotation().getY() * 180.0 / Math.PI);
                p.putDouble("rzDeg", pose.getRotation().getZ() * 180.0 / Math.PI);
            }
            // Object inputs are not automatically published (type-dependent).
        }
        return node;
    }

    /**
     * Returns a flattened list of all mechanisms owned by this composite (including nested children).
     */
    private static void flattenMechanisms(List<Mechanism> out, SuperstructureMechanism<?, ?> mech) {
        for (Child<?, ?> child : mech.children) {
            if (child.mechanism != null) {
                out.add(child.mechanism);
            }
            if (child.superstructure instanceof SuperstructureMechanism<?, ?>) {
                flattenMechanisms(out, (SuperstructureMechanism<?, ?>) child.superstructure);
            }
        }
    }

    /**
     * Typed accessors for common mechanism subclasses.
     */
        public Mechanisms getMechanisms() {
            return new Mechanisms();
        }

        public final class Mechanisms implements SuperstructureMechanismsView<SP> {
            @SuppressWarnings("unchecked")
            @Override
            public <E extends Enum<E> & SetpointProvider<Double>> StatefulArmMechanism<E> arm(Function<SP, E> mapper) {
            StatefulLike<E> mech = context.mechanism(mapper);
            if (mech instanceof StatefulArmMechanism<?>) {
                return (StatefulArmMechanism<E>) mech;
            }
            throw new IllegalArgumentException("Mapper does not resolve to an arm mechanism");
        }

        @SuppressWarnings("unchecked")
        @Override
        public <E extends Enum<E> & SetpointProvider<Double>> ElevatorMechanism.StatefulElevatorMechanism<E> elevator(Function<SP, E> mapper) {
            StatefulLike<E> mech = context.mechanism(mapper);
            if (mech instanceof ElevatorMechanism.StatefulElevatorMechanism<?>) {
                return (ElevatorMechanism.StatefulElevatorMechanism<E>) mech;
            }
            throw new IllegalArgumentException("Mapper does not resolve to an elevator mechanism");
        }

        @SuppressWarnings("unchecked")
        @Override
        public <E extends Enum<E> & SetpointProvider<Double>> TurretMechanism.StatefulTurretMechanism<E> turret(Function<SP, E> mapper) {
            StatefulLike<E> mech = context.mechanism(mapper);
            if (mech instanceof TurretMechanism.StatefulTurretMechanism<?>) {
                return (TurretMechanism.StatefulTurretMechanism<E>) mech;
            }
            throw new IllegalArgumentException("Mapper does not resolve to a turret mechanism");
        }

        @SuppressWarnings("unchecked")
        @Override
        public <E extends Enum<E> & SetpointProvider<Double>> StatefulFlywheelMechanism<E> flywheel(Function<SP, E> mapper) {
            StatefulLike<E> mech = context.mechanism(mapper);
            if (mech instanceof StatefulFlywheelMechanism<?>) {
                return (StatefulFlywheelMechanism<E>) mech;
            }
            throw new IllegalArgumentException("Mapper does not resolve to a flywheel mechanism");
        }

        @SuppressWarnings("unchecked")
        @Override
        public <E extends Enum<E> & SetpointProvider<Double>> StatefulMechanism<E> generic(Function<SP, E> mapper) {
            StatefulLike<E> mech = context.mechanism(mapper);
            if (mech instanceof StatefulMechanism<?>) {
                return (StatefulMechanism<E>) mech;
            }
            throw new IllegalArgumentException("Mapper does not resolve to a generic mechanism");
        }

        @Override
        public boolean input(String key) {
            return SuperstructureMechanism.this.input(key);
        }

        @Override
        public java.util.function.BooleanSupplier inputSupplier(String key) {
            return SuperstructureMechanism.this.inputSupplier(key);
        }

        @Override
        public double doubleInput(String key) {
            return SuperstructureMechanism.this.doubleInput(key);
        }

        @Override
        public DoubleSupplier doubleInputSupplier(String key) {
            return SuperstructureMechanism.this.doubleInputSupplier(key);
        }

        @Override
        public int intVal(String key) {
            return SuperstructureMechanism.this.intVal(key);
        }

        @Override
        public IntSupplier intValSupplier(String key) {
            return SuperstructureMechanism.this.intValSupplier(key);
        }

        @Override
        public String stringVal(String key) {
            return SuperstructureMechanism.this.stringVal(key);
        }

        @Override
        public Supplier<String> stringValSupplier(String key) {
            return SuperstructureMechanism.this.stringValSupplier(key);
        }

        @Override
        public Pose2d pose2dVal(String key) {
            return SuperstructureMechanism.this.pose2dVal(key);
        }

        @Override
        public Supplier<Pose2d> pose2dValSupplier(String key) {
            return SuperstructureMechanism.this.pose2dValSupplier(key);
        }

        @Override
        public Pose3d pose3dVal(String key) {
            return SuperstructureMechanism.this.pose3dVal(key);
        }

        @Override
        public Supplier<Pose3d> pose3dValSupplier(String key) {
            return SuperstructureMechanism.this.pose3dValSupplier(key);
        }

        @Override
        public <T> T objectInput(String key, Class<T> type) {
            return SuperstructureMechanism.this.objectInput(key, type);
        }

        @Override
        public <T> Supplier<T> objectInputSupplier(String key, Class<T> type) {
            return SuperstructureMechanism.this.objectInputSupplier(key, type);
        }

        /**
         * Returns the flattened list of all child mechanisms (including nested).
         */
        @Override
        public List<Mechanism> all() {
            List<Mechanism> list = new ArrayList<>();
            flattenMechanisms(list, SuperstructureMechanism.this);
            return list;
        }

        /**
         * Returns a nested superstructure resolved via the same mapper used in the parent config.
         */
        @Override
        public <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> SuperstructureMechanism<CS, CSP> superstructure(
                Function<SP, CS> mapper) {
            return SuperstructureMechanism.this.context.superstructure(mapper);
        }
    }

    @Override
    public List<Mechanism> flattenForRegistration() {
        return getMechanisms().all();
    }

    public void setRobotCore(RobotCore<?> robotCore) {
        this.robotCore = robotCore;
        if (robotCore != null) {
            appendDiagnosticLog("INFO", "lifecycle", "attached to RobotCore");
        }
        if (robotCore != null) {
            propagateRobotCore(robotCore, this);
        }
    }

    private void appendDiagnosticLog(String level, String category, String message) {
        if (message == null || message.isBlank()) {
            return;
        }
        String text = message.trim();
        String lvl = level != null && !level.isBlank() ? level : "INFO";
        String cat = category != null && !category.isBlank() ? category : "general";
        String systemKey = diagnosticsSystemKey();
        String systemName = systemKey.lastIndexOf('/') >= 0
                ? systemKey.substring(systemKey.lastIndexOf('/') + 1)
                : systemKey;
        String line = "[" + systemName + "] "
                + lvl.toLowerCase(java.util.Locale.ROOT)
                + " " + cat + ": " + text;
        diagnosticsLog.append((sequence, timestampSeconds) ->
                new DiagnosticsChannel.Event(sequence, timestampSeconds, systemKey, lvl, cat, text, line));
    }

    private static int sanitizeDiagnosticLogLimit(int requestedLimit) {
        if (requestedLimit <= 0) {
            return 0;
        }
        return Math.min(requestedLimit, DIAGNOSTIC_LOG_CAPACITY);
    }

    public List<DiagnosticsChannel.Event> getDiagnosticEvents(int requestedLimit) {
        int limit = sanitizeDiagnosticLogLimit(requestedLimit);
        return diagnosticsLog.snapshot(limit);
    }

    public int getDiagnosticLogCount() {
        return diagnosticsLog.count();
    }

    public void clearDiagnosticLog() {
        diagnosticsLog.clear();
    }

    public Map<String, Object> getDiagnosticsSummary() {
        Map<String, Object> summary = new LinkedHashMap<>();
        summary.put("name", getName());
        summary.put("systemKey", diagnosticsSystemKey());
        Enum<?> state = stateMachine.getGoalState();
        summary.put("state", state != null ? state.name() : "");
        summary.put("atGoal", stateMachine.atGoalState());
        summary.put("childrenAtGoals", childrenAtGoals());
        summary.put("childMechanismCount", getMechanisms().all().size());
        summary.put("logCount", getDiagnosticLogCount());
        return summary;
    }

    public Map<String, Object> getDiagnosticsSnapshot(int requestedLimit) {
        Map<String, Object> snapshot = new LinkedHashMap<>(getDiagnosticsSummary());
        int limit = sanitizeDiagnosticLogLimit(requestedLimit);
        snapshot.put("events", getDiagnosticEvents(limit));
        return snapshot;
    }

    public final class DiagnosticsView {
        public DiagnosticsView log(String level, String category, String message) {
            appendDiagnosticLog(level, category, message);
            return this;
        }

        public DiagnosticsView info(String category, String message) {
            appendDiagnosticLog("INFO", category, message);
            return this;
        }

        public DiagnosticsView warn(String category, String message) {
            appendDiagnosticLog("WARN", category, message);
            return this;
        }

        public DiagnosticsView error(String category, String message) {
            appendDiagnosticLog("ERROR", category, message);
            return this;
        }

        public List<DiagnosticsChannel.Event> events(int limit) {
            return getDiagnosticEvents(limit);
        }

        public int count() {
            return getDiagnosticLogCount();
        }

        public DiagnosticsView clear() {
            clearDiagnosticLog();
            return this;
        }

        public Map<String, Object> summary() {
            return getDiagnosticsSummary();
        }

        public Map<String, Object> snapshot(int limit) {
            return getDiagnosticsSnapshot(limit);
        }
    }

    /**
     * Assigns a dashboard owner path to all child mechanisms (including nested superstructures).
     *
     * <p>This is used by RobotCore to publish mechanisms under Athena/Mechanisms in a structured
     * tree (Owner -> Type -> Name) and avoid duplicate widgets across multiple tabs.</p>
     */
    public void assignDashboardOwners(String ownerPath) {
        String base = ownerPath;
        if (base == null || base.isBlank()) {
            String n = getName();
            base = (n == null || n.isBlank()) ? getClass().getSimpleName() : n;
        }
        propagateDashboardOwner(base, this);
    }

    public RobotCore<?> getRobotCore() {
        return robotCore;
    }

    SuperstructureMechanism<S, SP> setSourceConfig(SuperstructureConfig<?, ?> config) {
        this.sourceConfig = config;
        return this;
    }

    public SuperstructureConfig<?, ?> getSourceConfig() {
        return sourceConfig;
    }

    /**
     * Runs any init hooks registered on the {@link SuperstructureConfig}.
     *
     * <p>This is invoked by {@code RobotCore.robotInit()} after registration so hooks can safely
     * reference other mechanisms/superstructures via the context.</p>
     */
    @SuppressWarnings("unchecked")
    public void runInitHooks() {
        if (sourceConfig == null) {
            return;
        }
        SuperstructureConfig<S, SP> cfg = (SuperstructureConfig<S, SP>) sourceConfig;
        cfg.runInitHooks(context, stateMachine.getGoalState());
    }

    @SuppressWarnings("unchecked")
    public void runLifecycleHooks(RobotCoreHooks.Phase phase) {
        if (sourceConfig == null || phase == null) {
            return;
        }
        SuperstructureConfig<S, SP> cfg = (SuperstructureConfig<S, SP>) sourceConfig;
        cfg.runPhaseHooks(context, stateMachine.getGoalState(), phase);
    }

    /**
     * Returns a flattened list of this superstructure and all nested superstructures.
     */
    public List<SuperstructureMechanism<?, ?>> flattenSuperstructures() {
        List<SuperstructureMechanism<?, ?>> out = new ArrayList<>();
        flattenSuperstructures(out, this);
        return out;
    }

    private static void flattenSuperstructures(List<SuperstructureMechanism<?, ?>> out, SuperstructureMechanism<?, ?> mech) {
        if (mech == null) {
            return;
        }
        out.add(mech);
        for (Child<?, ?> child : mech.children) {
            if (child != null && child.superstructure != null) {
                flattenSuperstructures(out, child.superstructure);
            }
        }
    }

    private static void propagateRobotCore(RobotCore<?> robotCore, SuperstructureMechanism<?, ?> mech) {
        for (Child<?, ?> child : mech.children) {
            if (child.mechanism != null) {
                child.mechanism.setRobotCore(robotCore);
            }
            if (child.superstructure != null) {
                SuperstructureMechanism<?, ?> nested = child.superstructure;
                nested.robotCore = robotCore;
                propagateRobotCore(robotCore, nested);
            }
        }
    }

    private static void propagateDashboardOwner(String ownerPath, SuperstructureMechanism<?, ?> mech) {
        for (Child<?, ?> child : mech.children) {
            if (child.mechanism != null) {
                child.mechanism.setNetworkTablesOwnerPath(ownerPath);
            }
            if (child.superstructure != null) {
                SuperstructureMechanism<?, ?> nested = child.superstructure;
                String nestedName = nested.getName();
                if (nestedName == null || nestedName.isBlank()) {
                    nestedName = nested.getClass().getSimpleName();
                }
                propagateDashboardOwner(ownerPath + "/" + nestedName, nested);
            }
        }
    }

    public boolean input(String key) {
        return inputResolver.boolVal(key);
    }

    public java.util.function.BooleanSupplier inputSupplier(String key) {
        return inputResolver.boolSupplier(key);
    }

    public double doubleInput(String key) {
        return inputResolver.doubleVal(key);
    }

    public DoubleSupplier doubleInputSupplier(String key) {
        return inputResolver.doubleSupplier(key);
    }

    public int intVal(String key) {
        return inputResolver.intVal(key);
    }

    public IntSupplier intValSupplier(String key) {
        return inputResolver.intSupplier(key);
    }

    public String stringVal(String key) {
        return inputResolver.stringVal(key);
    }

    public Supplier<String> stringValSupplier(String key) {
        return inputResolver.stringSupplier(key);
    }

    public Pose2d pose2dVal(String key) {
        return inputResolver.pose2dVal(key);
    }

    public Supplier<Pose2d> pose2dValSupplier(String key) {
        return inputResolver.pose2dSupplier(key);
    }

    public Pose3d pose3dVal(String key) {
        return inputResolver.pose3dVal(key);
    }

    public Supplier<Pose3d> pose3dValSupplier(String key) {
        return inputResolver.pose3dSupplier(key);
    }

    public <T> T objectInput(String key, Class<T> type) {
        return inputResolver.objectVal(key, type);
    }

    public <T> Supplier<T> objectInputSupplier(String key, Class<T> type) {
        return inputResolver.objectSupplier(key, type);
    }

    private final class SuperstructureContextImpl implements SuperstructureContext<SP> {
        private SP overrideSetpoint;

        private void setOverrideSetpoint(SP setpoint) {
            this.overrideSetpoint = setpoint;
        }

        private void clearOverrideSetpoint() {
            this.overrideSetpoint = null;
        }

        @Override
        public SP setpoint() {
            SP override = overrideSetpoint;
            return override != null ? override : stateMachine.getGoalStateSetpoint();
        }

        @Override
        public RobotCore<?> robotCore() {
            if (SuperstructureMechanism.this.robotCore != null) {
                return SuperstructureMechanism.this.robotCore;
            }
            for (Mechanism mech : getMechanisms().all()) {
                RobotCore<?> core = mech.getRobotCore();
                if (core != null) {
                    return core;
                }
            }
            return null;
        }

        @Override
        @SuppressWarnings("unchecked")
        public <E extends Enum<E> & SetpointProvider<Double>> StatefulLike<E> mechanism(Function<SP, E> mapper) {
            Class<?> desiredType = null;
            SP setpoint = setpoint();
            if (setpoint != null) {
                E sample = mapper.apply(setpoint);
                if (sample != null) {
                    desiredType = sample.getDeclaringClass();
                }
            }

            for (Child<SP, ?> child : children) {
                if (child.mechanism == null) {
                    continue;
                }
                if (child.mapper == mapper || child.mapper.getClass().equals(mapper.getClass())) {
                    return (StatefulLike<E>) child.mechanism;
                }
                if (desiredType != null && desiredType.equals(child.stateType)) {
                    return (StatefulLike<E>) child.mechanism;
                }
            }
            throw new IllegalArgumentException("No child mechanism found for mapper");
        }

        @Override
        @SuppressWarnings("unchecked")
        public <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> SuperstructureMechanism<CS, CSP> superstructure(
                Function<SP, CS> mapper) {
            Class<?> desiredType = null;
            SP setpoint = setpoint();
            if (setpoint != null) {
                CS sample = mapper.apply(setpoint);
                if (sample != null) {
                    desiredType = sample.getDeclaringClass();
                }
            }

            for (Child<SP, ?> child : children) {
                if (child.superstructure == null) {
                    continue;
                }
                if (child.mapper == mapper || child.mapper.getClass().equals(mapper.getClass())) {
                    return (SuperstructureMechanism<CS, CSP>) child.superstructure;
                }
                if (desiredType != null && desiredType.equals(child.stateType)) {
                    return (SuperstructureMechanism<CS, CSP>) child.superstructure;
                }
            }
            throw new IllegalArgumentException("No child superstructure found for mapper");
        }

        @Override
        public SuperstructureMechanismsView<SP> getMechanisms() {
            return SuperstructureMechanism.this.getMechanisms();
        }

        @Override
        public boolean input(String key) {
            return SuperstructureMechanism.this.input(key);
        }

        @Override
        public double doubleInput(String key) {
            return SuperstructureMechanism.this.doubleInput(key);
        }

        @Override
        public DoubleSupplier doubleInputSupplier(String key) {
            return SuperstructureMechanism.this.doubleInputSupplier(key);
        }

        @Override
        public int intVal(String key) {
            return SuperstructureMechanism.this.intVal(key);
        }

        @Override
        public IntSupplier intValSupplier(String key) {
            return SuperstructureMechanism.this.intValSupplier(key);
        }

        @Override
        public String stringVal(String key) {
            return SuperstructureMechanism.this.stringVal(key);
        }

        @Override
        public Supplier<String> stringValSupplier(String key) {
            return SuperstructureMechanism.this.stringValSupplier(key);
        }

        @Override
        public Pose2d pose2dVal(String key) {
            return SuperstructureMechanism.this.pose2dVal(key);
        }

        @Override
        public Supplier<Pose2d> pose2dValSupplier(String key) {
            return SuperstructureMechanism.this.pose2dValSupplier(key);
        }

        @Override
        public Pose3d pose3dVal(String key) {
            return SuperstructureMechanism.this.pose3dVal(key);
        }

        @Override
        public Supplier<Pose3d> pose3dValSupplier(String key) {
            return SuperstructureMechanism.this.pose3dValSupplier(key);
        }

        @Override
        public <T> T objectInput(String key, Class<T> type) {
            return SuperstructureMechanism.this.objectInput(key, type);
        }

        @Override
        public <T> Supplier<T> objectInputSupplier(String key, Class<T> type) {
            return SuperstructureMechanism.this.objectInputSupplier(key, type);
        }

        @Override
        public <E extends Enum<E> & SetpointProvider<Double>> double mappedSetpoint(Function<SP, E> mapper) {
            SP setpoint = setpoint();
            if (setpoint == null) {
                return Double.NaN;
            }
            E state = mapper.apply(setpoint);
            if (state == null) {
                return Double.NaN;
            }
            return state.getSetpoint();
        }

    }
}
