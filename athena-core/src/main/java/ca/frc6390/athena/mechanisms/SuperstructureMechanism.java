package ca.frc6390.athena.mechanisms;

import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
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
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.Queue;

/**
 * Composite mechanism that coordinates multiple stateful mechanisms using a superstate enum.
 * Constraints reference child mechanisms via {@link SuperstructureContext#mechanisms()} using
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
            this.stateMachine = castMachine(stateful.stateMachine().machine());
            this.mapper = castMapper(mapper);
            this.stateType = stateType;
        }

        <CS extends Enum<CS> & SetpointProvider<CSP>, CSP> Child(SuperstructureMechanism<CS, CSP> superstructure, Function<SP, CS> mapper, Class<CS> stateType) {
            this.mechanism = null;
            this.superstructure = superstructure;
            this.stateMachine = castMachine(superstructure.stateMachine().machine());
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
    private final Queue<SP> queuedSetpoints = new ArrayDeque<>();
    private SP queuedSetpoint;
    private SP lastAppliedSetpoint;
    private S prevState;
    private String ntCommandsPath;
    private NetworkTableEntry ntCustomSetpointEntry;
    private NtSendablePublisher ntStateChooserPublisher;
    private NtSendablePublisher ntModeChooserPublisher;
    private NtCommandButton ntRequestButton;
    private NtCommandButton ntQueueButton;
    private NtCommandButton ntClearQueueButton;
    private final SendableChooser<ControlCommandMode> ntCommandModeChooser = createControlCommandModeChooser();
    private final BoundedEventLog<DiagnosticsChannel.Event> diagnosticsLog =
            new BoundedEventLog<>(DIAGNOSTIC_LOG_CAPACITY);
    private final DiagnosticsView diagnosticsView = new DiagnosticsView();
    private final StateMachineSection<S, SP> stateMachineSection;
    private final InputSection inputSection;
    private final ChildrenSection<SP> childrenSection;
    private final NetworkTablesSection networkTablesSection;
    private final SuperstructureMechanismsView<SP> mechanismsView;
    private final List<Mechanism> flattenedMechanisms;
    private final List<SuperstructureMechanism<?, ?>> flattenedSuperstructures;
    private final int flattenedMechanismCount;
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
        this.stateMachine = new StateMachine<>(initialState, this::childrenAtGoalsInternal);
        this.stateMachine.setAtStateDelay(stateMachineDelaySeconds);
        this.context = new SuperstructureContextImpl();
        this.stateMachineSection = new StateMachineSection<>(this);
        this.inputSection = new InputSection(this);
        this.childrenSection = new ChildrenSection<>(this);
        this.networkTablesSection = new NetworkTablesSection(this);
        this.mechanismsView = new MechanismsView();
        List<Mechanism> allMechanisms = new ArrayList<>();
        flattenMechanisms(allMechanisms, this);
        this.flattenedMechanisms = List.copyOf(allMechanisms);
        this.flattenedMechanismCount = this.flattenedMechanisms.size();
        List<SuperstructureMechanism<?, ?>> allSuperstructures = new ArrayList<>();
        flattenSuperstructures(allSuperstructures, this);
        this.flattenedSuperstructures = List.copyOf(allSuperstructures);
        lastAppliedSetpoint = initialState.getSetpoint();
        applySetpoints(lastAppliedSetpoint);
        this.prevState = initialState;
    }

    public SuperstructureMechanism<S, SP> stateMachine(Consumer<StateMachineSection<S, SP>> section) {
        if (section != null) {
            section.accept(stateMachineSection);
        }
        return this;
    }

    public StateMachineSection<S, SP> stateMachine() {
        return stateMachineSection;
    }

    public SuperstructureMechanism<S, SP> input(Consumer<InputSection> section) {
        if (section != null) {
            section.accept(inputSection);
        }
        return this;
    }

    public InputSection input() {
        return inputSection;
    }

    public SuperstructureMechanism<S, SP> children(Consumer<ChildrenSection<SP>> section) {
        if (section != null) {
            section.accept(childrenSection);
        }
        return this;
    }

    public ChildrenSection<SP> children() {
        return childrenSection;
    }

    public SuperstructureMechanism<S, SP> mechanisms(Consumer<SuperstructureMechanismsView<SP>> section) {
        if (section != null) {
            section.accept(mechanismsView);
        }
        return this;
    }

    public SuperstructureMechanismsView<SP> mechanisms() {
        return mechanismsView;
    }

    public SuperstructureMechanism<S, SP> networkTables(Consumer<NetworkTablesSection> section) {
        if (section != null) {
            section.accept(networkTablesSection);
        }
        return this;
    }

    public NetworkTablesSection networkTables() {
        return networkTablesSection;
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

    private void queueStateInternal(S state) {
        if (state != null) {
            appendDiagnosticLog("INFO", "state", "queued state '" + state.name() + "'");
        }
        SuperstructureConfig.Constraint<S, SP> constraint = constraints.get(state);
        if (constraint == null) {
            stateMachine.queueState(state);
            return;
        }
        BooleanSupplier guard = () -> constraint.guard.test(context);
        if (!constraint.transitionStates.isEmpty() && !guard.getAsBoolean()) {
            for (S transition : constraint.transitionStates) {
                stateMachine.queueState(transition);
            }
        }
        stateMachine.queueState(state, guard);
    }

    private void queueSetpointInternal(SP setpoint) {
        if (setpoint == null) {
            return;
        }
        queuedSetpoints.add(setpoint);
    }

    private void requestSetpointInternal(SP setpoint) {
        if (setpoint == null) {
            return;
        }
        stateMachine.resetQueue();
        queuedSetpoints.clear();
        queuedSetpoint = setpoint;
    }

    private void requestStateInternal(S target) {
        if (target == null) {
            return;
        }
        queuedSetpoint = null;
        queuedSetpoints.clear();
        stateMachine.requestState(target);
    }

    private void clearStateMachineInternal() {
        queuedSetpoint = null;
        queuedSetpoints.clear();
        stateMachine.resetQueue();
    }

    private boolean childrenAtGoalsInternal() {
        for (Child<SP, ?> child : children) {
            if (child.mechanism instanceof StatefulLike<?> stateful) {
                if (!stateful.stateMachine().atGoal()) {
                    return false;
                }
            } else if (child.superstructure != null) {
                if (!child.superstructure.stateMachine().atGoal()) {
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
        updateQueuedSetpoint();
        applySetpointsIfChanged();
        applyAttachments();
        S current = stateMachine.getGoalState();
        boolean changed = !Objects.equals(current, prevState);
        S from = prevState;
        if (changed) {
            applyExitBindings(from);
            applySetpointsIfChanged();
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

    private SP currentSetpoint() {
        return queuedSetpoint != null ? queuedSetpoint : stateMachine.getGoalStateSetpoint();
    }

    /**
     * Setpoint that should currently be pushed to child mechanisms/superstructures.
     *
     * <p>When a new superstate is queued, we drive children toward that next state's tuple
     * immediately (instead of waiting for the parent goal to flip). This prevents top-level
     * superstructures from stalling at initial/off tuples when parent transitions are gated on
     * child at-goal conditions.</p>
     */
    private SP desiredChildSetpoint() {
        if (queuedSetpoint != null) {
            return queuedSetpoint;
        }
        S goal = stateMachine.getGoalState();
        S next = stateMachine.getNextState();
        if (next != null && !Objects.equals(next, goal)) {
            SP nextSetpoint = next.getSetpoint();
            if (nextSetpoint != null) {
                return nextSetpoint;
            }
        }
        return stateMachine.getGoalStateSetpoint();
    }

    private void applySetpointsIfChanged() {
        SP desired = desiredChildSetpoint();
        if (Objects.equals(lastAppliedSetpoint, desired)) {
            return;
        }
        applySetpoints(desired);
        lastAppliedSetpoint = desired;
    }

    private void updateQueuedSetpoint() {
        if (!stateMachine.atGoalState()) {
            return;
        }
        if (queuedSetpoint == null) {
            if (!queuedSetpoints.isEmpty()) {
                queuedSetpoint = queuedSetpoints.poll();
            }
            return;
        }
        queuedSetpoint = queuedSetpoints.poll();
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
            mech.visualization().rootOverride(pose);
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
                "Enable section flags under " + node.path() + "/NetworkTableConfig to publish more topics.");

        RobotNetworkTables.MechanismToggles toggles = nt.superstructureConfig(node);

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
        if (toggles.controlEnabled()) {
            processNetworkTableCommands(node);
        }
        publishNestedSuperstructures(node);
        return node;
    }

    private void publishNestedSuperstructures(RobotNetworkTables.Node node) {
        for (Child<SP, ?> child : children) {
            if (child == null || child.superstructure == null) {
                continue;
            }
            SuperstructureMechanism<?, ?> nested = child.superstructure;
            String nestedName = nested.getName();
            if (nestedName == null || nestedName.isBlank()) {
                nestedName = nested.getClass().getSimpleName();
            }
            nested.networkTables(node.child(nestedName));
        }
    }

    private void processNetworkTableCommands(RobotNetworkTables.Node node) {
        if (node == null) {
            return;
        }
        RobotNetworkTables.Node commands = node.child("Control").child("Commands");
        ensureNetworkTableCommandEntries(commands);
        updateCommandButton(ntRequestButton);
        updateCommandButton(ntQueueButton);
        updateCommandButton(ntClearQueueButton);
        updateSendablePublisher(ntStateChooserPublisher);
        updateSendablePublisher(ntModeChooserPublisher);
    }

    private void ensureNetworkTableCommandEntries(RobotNetworkTables.Node commands) {
        String path = commands.path();
        boolean samePath = (path == null && ntCommandsPath == null)
                || (path != null && path.equals(ntCommandsPath));
        if (samePath && hasNetworkTableCommandEntries()) {
            return;
        }
        closeCommandButton(ntRequestButton);
        closeCommandButton(ntQueueButton);
        closeCommandButton(ntClearQueueButton);
        closeSendablePublisher(ntStateChooserPublisher);
        closeSendablePublisher(ntModeChooserPublisher);
        ntRequestButton = null;
        ntQueueButton = null;
        ntClearQueueButton = null;
        ntStateChooserPublisher = null;
        ntModeChooserPublisher = null;
        ntCommandsPath = path;
        ntCustomSetpointEntry = commands.entry("CustomSetpoint");
        ntStateChooserPublisher = publishSendable(
                ntStateChooserPublisher,
                commands.child("StateChooser"),
                stateMachine.chooser());
        ntModeChooserPublisher = publishSendable(
                ntModeChooserPublisher,
                commands.child("ModeChooser"),
                ntCommandModeChooser);
        ntRequestButton = createCommandButton(
                commands.child("Request"),
                "Request",
                Commands.runOnce(this::requestFromChooserOrCustom).ignoringDisable(true));
        ntQueueButton = createCommandButton(
                commands.child("Queue"),
                "Queue",
                Commands.runOnce(this::queueFromChooserOrCustom).ignoringDisable(true));
        ntClearQueueButton = createCommandButton(
                commands.child("ClearAll"),
                "ClearAll",
                Commands.runOnce(this::clearStateMachineInternal).ignoringDisable(true));
        initIfAbsent(ntCustomSetpointEntry, formatSetpoint(currentSetpoint()));
    }

    private boolean hasNetworkTableCommandEntries() {
        return ntCustomSetpointEntry != null
                && ntStateChooserPublisher != null
                && ntModeChooserPublisher != null
                && ntRequestButton != null
                && ntQueueButton != null
                && ntClearQueueButton != null;
    }

    private void requestFromChooserOrCustom() {
        if (selectedControlCommandMode() == ControlCommandMode.CUSTOM_SETPOINT) {
            requestCustomSetpoint();
            return;
        }
        S selected = selectedState();
        if (selected != null) {
            requestStateInternal(selected);
        }
    }

    private void queueFromChooserOrCustom() {
        if (selectedControlCommandMode() == ControlCommandMode.CUSTOM_SETPOINT) {
            queueCustomSetpoint();
            return;
        }
        S selected = selectedState();
        if (selected != null) {
            queueStateInternal(selected);
        }
    }

    private ControlCommandMode selectedControlCommandMode() {
        ControlCommandMode selected = ntCommandModeChooser.getSelected();
        return selected != null ? selected : ControlCommandMode.STATE;
    }

    private boolean requestCustomSetpoint() {
        SP parsed = parseCustomSetpoint(readCustomSetpointInput());
        if (parsed != null) {
            requestSetpointInternal(parsed);
            return true;
        }
        return false;
    }

    private boolean queueCustomSetpoint() {
        SP parsed = parseCustomSetpoint(readCustomSetpointInput());
        if (parsed != null) {
            queueSetpointInternal(parsed);
            return true;
        }
        return false;
    }

    private String readCustomSetpointInput() {
        if (ntCustomSetpointEntry == null) {
            return "";
        }
        return ntCustomSetpointEntry.getString("");
    }

    private S selectedState() {
        S selected = stateMachine.chooser().getSelected();
        if (selected != null) {
            return selected;
        }
        return stateMachine.getGoalState();
    }

    @SuppressWarnings("unchecked")
    private SP parseCustomSetpoint(String rawValue) {
        String trimmed = rawValue != null ? rawValue.trim() : "";
        if (trimmed.isEmpty()) {
            return null;
        }
        SP template = currentSetpoint();
        if (template == null) {
            appendDiagnosticLog("WARN", "networkTables", "cannot parse custom setpoint without a setpoint template");
            return null;
        }
        try {
            if (template instanceof Double) {
                double value = parseFiniteDouble(trimmed);
                return (SP) Double.valueOf(value);
            }
            if (template instanceof Float) {
                double value = parseFiniteDouble(trimmed);
                return (SP) Float.valueOf((float) value);
            }
            if (template instanceof Integer) {
                return (SP) Integer.valueOf(Integer.parseInt(trimmed));
            }
            if (template instanceof Long) {
                return (SP) Long.valueOf(Long.parseLong(trimmed));
            }
            if (template instanceof Short) {
                return (SP) Short.valueOf(Short.parseShort(trimmed));
            }
            if (template instanceof Byte) {
                return (SP) Byte.valueOf(Byte.parseByte(trimmed));
            }
            if (template instanceof Boolean) {
                if ("true".equalsIgnoreCase(trimmed) || "1".equals(trimmed)) {
                    return (SP) Boolean.TRUE;
                }
                if ("false".equalsIgnoreCase(trimmed) || "0".equals(trimmed)) {
                    return (SP) Boolean.FALSE;
                }
                appendDiagnosticLog("WARN", "networkTables", "invalid boolean custom setpoint '" + rawValue + "'");
                return null;
            }
            if (template instanceof String) {
                return (SP) trimmed;
            }
        } catch (NumberFormatException ex) {
            appendDiagnosticLog("WARN", "networkTables", "invalid custom setpoint '" + rawValue + "'");
            return null;
        }
        appendDiagnosticLog(
                "WARN",
                "networkTables",
                "unsupported custom setpoint type '" + template.getClass().getSimpleName() + "'");
        return null;
    }

    private static double parseFiniteDouble(String value) {
        double parsed = Double.parseDouble(value);
        if (!Double.isFinite(parsed)) {
            throw new NumberFormatException("Non-finite value");
        }
        return parsed;
    }

    private static SendableChooser<ControlCommandMode> createControlCommandModeChooser() {
        SendableChooser<ControlCommandMode> chooser = new SendableChooser<>();
        chooser.setDefaultOption("State", ControlCommandMode.STATE);
        chooser.addOption("CustomSetpoint", ControlCommandMode.CUSTOM_SETPOINT);
        return chooser;
    }

    private static String formatSetpoint(Object value) {
        return value != null ? String.valueOf(value) : "";
    }

    private static void initIfAbsent(NetworkTableEntry entry, boolean value) {
        if (entry != null && entry.getType() == NetworkTableType.kUnassigned) {
            entry.setBoolean(value);
        }
    }

    private static void initIfAbsent(NetworkTableEntry entry, String value) {
        if (entry != null && entry.getType() == NetworkTableType.kUnassigned) {
            entry.setString(value != null ? value : "");
        }
    }

    private static NtCommandButton createCommandButton(RobotNetworkTables.Node node, String name, Command command) {
        if (node == null || command == null) {
            return null;
        }
        Command named = command.withName(name != null ? name : command.getName());
        return new NtCommandButton(node, named);
    }

    private enum ControlCommandMode {
        STATE,
        CUSTOM_SETPOINT
    }

    private static NtSendablePublisher publishSendable(
            NtSendablePublisher existing,
            RobotNetworkTables.Node node,
            Sendable sendable) {
        if (existing != null) {
            existing.close();
        }
        if (node == null || sendable == null) {
            return null;
        }
        return new NtSendablePublisher(node, sendable);
    }

    private static void updateSendablePublisher(NtSendablePublisher publisher) {
        if (publisher != null) {
            publisher.update();
        }
    }

    private static void closeSendablePublisher(NtSendablePublisher publisher) {
        if (publisher != null) {
            publisher.close();
        }
    }

    private static void closeCommandButton(NtCommandButton button) {
        if (button != null) {
            button.close();
        }
    }

    private static void updateCommandButton(NtCommandButton button) {
        if (button != null) {
            button.update();
        }
    }

    private static final class NtCommandButton {
        private final SendableBuilderImpl builder;

        private NtCommandButton(RobotNetworkTables.Node node, Command command) {
            builder = new SendableBuilderImpl();
            builder.setTable(node.table());
            command.initSendable(builder);
            builder.startListeners();
            builder.update();
        }

        private void update() {
            builder.update();
        }

        private void close() {
            builder.close();
        }
    }

    private static final class NtSendablePublisher {
        private final SendableBuilderImpl builder;

        private NtSendablePublisher(RobotNetworkTables.Node node, Sendable sendable) {
            builder = new SendableBuilderImpl();
            builder.setTable(node.table());
            sendable.initSendable(builder);
            builder.startListeners();
            builder.update();
        }

        private void update() {
            builder.update();
        }

        private void close() {
            builder.close();
        }
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
    private final class MechanismsView implements SuperstructureMechanismsView<SP> {
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
            return SuperstructureMechanism.this.input().bool(key);
        }

        @Override
        public java.util.function.BooleanSupplier inputSupplier(String key) {
            return SuperstructureMechanism.this.input().boolSupplier(key);
        }

        @Override
        public double doubleInput(String key) {
            return SuperstructureMechanism.this.input().dbl(key);
        }

        @Override
        public DoubleSupplier doubleInputSupplier(String key) {
            return SuperstructureMechanism.this.input().dblSupplier(key);
        }

        @Override
        public int intVal(String key) {
            return SuperstructureMechanism.this.input().integer(key);
        }

        @Override
        public IntSupplier intValSupplier(String key) {
            return SuperstructureMechanism.this.input().integerSupplier(key);
        }

        @Override
        public String stringVal(String key) {
            return SuperstructureMechanism.this.input().string(key);
        }

        @Override
        public Supplier<String> stringValSupplier(String key) {
            return SuperstructureMechanism.this.input().stringSupplier(key);
        }

        @Override
        public Pose2d pose2dVal(String key) {
            return SuperstructureMechanism.this.input().pose2d(key);
        }

        @Override
        public Supplier<Pose2d> pose2dValSupplier(String key) {
            return SuperstructureMechanism.this.input().pose2dSupplier(key);
        }

        @Override
        public Pose3d pose3dVal(String key) {
            return SuperstructureMechanism.this.input().pose3d(key);
        }

        @Override
        public Supplier<Pose3d> pose3dValSupplier(String key) {
            return SuperstructureMechanism.this.input().pose3dSupplier(key);
        }

        @Override
        public <T> T objectInput(String key, Class<T> type) {
            return SuperstructureMechanism.this.input().object(key, type);
        }

        @Override
        public <T> Supplier<T> objectInputSupplier(String key, Class<T> type) {
            return SuperstructureMechanism.this.input().objectSupplier(key, type);
        }

        /**
         * Returns the flattened list of all child mechanisms (including nested).
         */
        @Override
        public List<Mechanism> all() {
            return flattenedMechanisms;
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

    public static final class ChildrenSection<SP> {
        private final SuperstructureMechanism<?, SP> owner;

        private ChildrenSection(SuperstructureMechanism<?, SP> owner) {
            this.owner = owner;
        }

        public List<Mechanism> all() {
            return owner.flattenedMechanisms;
        }

        public List<SuperstructureMechanism<?, ?>> superstructures() {
            return owner.flattenedSuperstructures;
        }

        public <T> T key(String name, Class<T> type) {
            Objects.requireNonNull(type, "type");
            if (name == null || name.isBlank()) {
                return null;
            }
            if (SuperstructureMechanism.class.isAssignableFrom(type)) {
                for (SuperstructureMechanism<?, ?> superstructure : superstructures()) {
                    if (superstructure == null || superstructure.getName() == null) {
                        continue;
                    }
                    if (!name.equals(superstructure.getName())) {
                        continue;
                    }
                    if (!type.isInstance(superstructure)) {
                        throw new IllegalArgumentException("Child superstructure '" + name + "' is not a "
                                + type.getSimpleName() + " (was " + superstructure.getClass().getSimpleName() + ")");
                    }
                    return type.cast(superstructure);
                }
                return null;
            }
            for (Mechanism mechanism : all()) {
                if (mechanism == null || mechanism.getName() == null) {
                    continue;
                }
                if (!name.equals(mechanism.getName())) {
                    continue;
                }
                if (!type.isInstance(mechanism)) {
                    throw new IllegalArgumentException("Child mechanism '" + name + "' is not a "
                            + type.getSimpleName() + " (was " + mechanism.getClass().getSimpleName() + ")");
                }
                return type.cast(mechanism);
            }
            return null;
        }

        public <T> T key(Enum<?> key, Class<T> type) {
            Objects.requireNonNull(key, "key");
            return key(key.name(), type);
        }

        public <T> T key(MechanismConfig<?> config, Class<T> type) {
            Objects.requireNonNull(type, "type");
            if (config == null) {
                return null;
            }
            for (Mechanism mechanism : all()) {
                if (mechanism == null || mechanism.getSourceConfig() != config) {
                    continue;
                }
                if (!type.isInstance(mechanism)) {
                    throw new IllegalArgumentException("Child mechanism built from config is not a "
                            + type.getSimpleName() + " (was " + mechanism.getClass().getSimpleName() + ")");
                }
                return type.cast(mechanism);
            }
            return null;
        }

        public <T> T key(SuperstructureConfig<?, ?> config, Class<T> type) {
            Objects.requireNonNull(type, "type");
            if (config == null) {
                return null;
            }
            for (SuperstructureMechanism<?, ?> superstructure : superstructures()) {
                if (superstructure == null || superstructure.getSourceConfig() != config) {
                    continue;
                }
                if (!type.isInstance(superstructure)) {
                    throw new IllegalArgumentException("Child superstructure built from config is not a "
                            + type.getSimpleName() + " (was " + superstructure.getClass().getSimpleName() + ")");
                }
                return type.cast(superstructure);
            }
            return null;
        }

        public SuperstructureMechanism<?, ?> superstructure(String name) {
            return key(name, SuperstructureMechanism.class);
        }

        public SuperstructureMechanism<?, ?> superstructure(Enum<?> key) {
            return key(key, SuperstructureMechanism.class);
        }

        public <S extends Enum<S> & SetpointProvider<CSP>, CSP> SuperstructureMechanism<S, CSP> superstructure(
                SuperstructureConfig<S, CSP> config) {
            return key(config, SuperstructureMechanism.class);
        }

        public TurretMechanism turret(String name) {
            return key(name, TurretMechanism.class);
        }

        public TurretMechanism turret(Enum<?> key) {
            return key(key, TurretMechanism.class);
        }

        public TurretMechanism turret(MechanismConfig<?> config) {
            return key(config, TurretMechanism.class);
        }

        public ElevatorMechanism elevator(String name) {
            return key(name, ElevatorMechanism.class);
        }

        public ElevatorMechanism elevator(Enum<?> key) {
            return key(key, ElevatorMechanism.class);
        }

        public ElevatorMechanism elevator(MechanismConfig<?> config) {
            return key(config, ElevatorMechanism.class);
        }

        public ArmMechanism arm(String name) {
            return key(name, ArmMechanism.class);
        }

        public ArmMechanism arm(Enum<?> key) {
            return key(key, ArmMechanism.class);
        }

        public ArmMechanism arm(MechanismConfig<?> config) {
            return key(config, ArmMechanism.class);
        }

        public FlywheelMechanism flywheel(String name) {
            return key(name, FlywheelMechanism.class);
        }

        public FlywheelMechanism flywheel(Enum<?> key) {
            return key(key, FlywheelMechanism.class);
        }

        public FlywheelMechanism flywheel(MechanismConfig<?> config) {
            return key(config, FlywheelMechanism.class);
        }

        public Mechanism generic(String name) {
            return key(name, Mechanism.class);
        }

        public Mechanism generic(Enum<?> key) {
            return key(key, Mechanism.class);
        }

        public Mechanism generic(MechanismConfig<?> config) {
            return key(config, Mechanism.class);
        }
    }

    @Override
    public List<Mechanism> flattenForRegistration() {
        return children().all();
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

    private List<DiagnosticsChannel.Event> diagnosticEvents(int requestedLimit) {
        int limit = sanitizeDiagnosticLogLimit(requestedLimit);
        return diagnosticsLog.snapshot(limit);
    }

    private int diagnosticLogCount() {
        return diagnosticsLog.count();
    }

    private void clearDiagnosticLog() {
        diagnosticsLog.clear();
    }

    private Map<String, Object> diagnosticsSummary() {
        Map<String, Object> summary = new LinkedHashMap<>();
        summary.put("name", getName());
        summary.put("systemKey", diagnosticsSystemKey());
        Enum<?> state = stateMachine.getGoalState();
        summary.put("state", state != null ? state.name() : "");
        summary.put("atGoal", stateMachine.atGoalState());
        summary.put("childMechanismCount", flattenedMechanismCount);
        summary.put("logCount", diagnosticLogCount());
        return summary;
    }

    private Map<String, Object> diagnosticsSnapshot(int requestedLimit) {
        Map<String, Object> snapshot = new LinkedHashMap<>(diagnosticsSummary());
        int limit = sanitizeDiagnosticLogLimit(requestedLimit);
        snapshot.put("events", diagnosticEvents(limit));
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
            return diagnosticEvents(limit);
        }

        public int count() {
            return diagnosticLogCount();
        }

        public DiagnosticsView clear() {
            clearDiagnosticLog();
            return this;
        }

        public Map<String, Object> summary() {
            return diagnosticsSummary();
        }

        public Map<String, Object> snapshot(int limit) {
            return diagnosticsSnapshot(limit);
        }
    }

    public static final class NetworkTablesSection {
        private final SuperstructureMechanism<?, ?> owner;

        private NetworkTablesSection(SuperstructureMechanism<?, ?> owner) {
            this.owner = owner;
        }

        public NetworkTablesSection ownerPath(String ownerPath) {
            owner.assignDashboardOwnerPath(ownerPath);
            return this;
        }
    }

    public static final class InputSection {
        private final SuperstructureMechanism<?, ?> owner;

        private InputSection(SuperstructureMechanism<?, ?> owner) {
            this.owner = owner;
        }

        public boolean bool(String key) {
            return owner.boolInput(key);
        }

        public BooleanSupplier boolSupplier(String key) {
            return owner.boolInputSupplier(key);
        }

        public double dbl(String key) {
            return owner.doubleInputValue(key);
        }

        public DoubleSupplier dblSupplier(String key) {
            return owner.doubleInputSupplierValue(key);
        }

        public int integer(String key) {
            return owner.intInputValue(key);
        }

        public IntSupplier integerSupplier(String key) {
            return owner.intInputSupplierValue(key);
        }

        public String string(String key) {
            return owner.stringInputValue(key);
        }

        public Supplier<String> stringSupplier(String key) {
            return owner.stringInputSupplierValue(key);
        }

        public Pose2d pose2d(String key) {
            return owner.pose2dInputValue(key);
        }

        public Supplier<Pose2d> pose2dSupplier(String key) {
            return owner.pose2dInputSupplierValue(key);
        }

        public Pose3d pose3d(String key) {
            return owner.pose3dInputValue(key);
        }

        public Supplier<Pose3d> pose3dSupplier(String key) {
            return owner.pose3dInputSupplierValue(key);
        }

        public <T> T object(String key, Class<T> type) {
            return owner.objectInputValue(key, type);
        }

        public <T> Supplier<T> objectSupplier(String key, Class<T> type) {
            return owner.objectInputSupplierValue(key, type);
        }
    }

    public static final class StateMachineSection<S extends Enum<S> & SetpointProvider<SP>, SP> {
        private final SuperstructureMechanism<S, SP> owner;

        private StateMachineSection(SuperstructureMechanism<S, SP> owner) {
            this.owner = owner;
        }

        public StateMachineSection<S, SP> queue(S state) {
            owner.queueStateInternal(state);
            return this;
        }

        public StateMachineSection<S, SP> queue(SP setpoint) {
            owner.queueSetpointInternal(setpoint);
            return this;
        }

        public StateMachineSection<S, SP> request(S target) {
            owner.requestStateInternal(target);
            return this;
        }

        public StateMachineSection<S, SP> request(SP setpoint) {
            owner.requestSetpointInternal(setpoint);
            return this;
        }

        public StateMachineSection<S, SP> clear() {
            owner.clearStateMachineInternal();
            return this;
        }

        public StateMachineSection<S, SP> graph(StateGraph<S> graph) {
            owner.stateMachine.setStateGraph(graph);
            return this;
        }

        public S goal() {
            return owner.stateMachine.getGoalState();
        }

        public S next() {
            return owner.stateMachine.getNextState();
        }

        public String queue() {
            return owner.stateMachine.getNextStateQueue();
        }

        public boolean atGoal() {
            return owner.stateMachine.atGoalState();
        }

        @SafeVarargs
        public final boolean at(S... states) {
            return owner.stateMachine.atState(states);
        }

        public SP setpoint() {
            return owner.currentSetpoint();
        }

        public StateGraph<S> graph() {
            return owner.stateMachine.getStateGraph();
        }

        public StateMachine<SP, S> machine() {
            return owner.stateMachine;
        }

        public StateMachineSnapshot<SP, S> snapshot() {
            return new StateMachineSnapshot<>(
                    goal(),
                    next(),
                    queue(),
                    atGoal(),
                    setpoint(),
                    graph());
        }
    }

    public record StateMachineSnapshot<SP, S extends Enum<S> & SetpointProvider<SP>>(
            S goal,
            S next,
            String queue,
            boolean atGoal,
            SP setpoint,
            StateGraph<S> graph) {}

    private void assignDashboardOwnerPath(String ownerPath) {
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
                child.mechanism.networkTables().ownerPath(ownerPath);
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

    private boolean boolInput(String key) {
        return inputResolver.boolVal(key);
    }

    private BooleanSupplier boolInputSupplier(String key) {
        return inputResolver.boolSupplier(key);
    }

    private double doubleInputValue(String key) {
        return inputResolver.doubleVal(key);
    }

    private DoubleSupplier doubleInputSupplierValue(String key) {
        return inputResolver.doubleSupplier(key);
    }

    private int intInputValue(String key) {
        return inputResolver.intVal(key);
    }

    private IntSupplier intInputSupplierValue(String key) {
        return inputResolver.intSupplier(key);
    }

    private String stringInputValue(String key) {
        return inputResolver.stringVal(key);
    }

    private Supplier<String> stringInputSupplierValue(String key) {
        return inputResolver.stringSupplier(key);
    }

    private Pose2d pose2dInputValue(String key) {
        return inputResolver.pose2dVal(key);
    }

    private Supplier<Pose2d> pose2dInputSupplierValue(String key) {
        return inputResolver.pose2dSupplier(key);
    }

    private Pose3d pose3dInputValue(String key) {
        return inputResolver.pose3dVal(key);
    }

    private Supplier<Pose3d> pose3dInputSupplierValue(String key) {
        return inputResolver.pose3dSupplier(key);
    }

    private <T> T objectInputValue(String key, Class<T> type) {
        return inputResolver.objectVal(key, type);
    }

    private <T> Supplier<T> objectInputSupplierValue(String key, Class<T> type) {
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
            return override != null ? override : currentSetpoint();
        }

        @Override
        public RobotCore<?> robotCore() {
            if (SuperstructureMechanism.this.robotCore != null) {
                return SuperstructureMechanism.this.robotCore;
            }
            for (Mechanism mech : mechanisms().all()) {
                RobotCore<?> core = mech.getRobotCore();
                if (core != null) {
                    return core;
                }
            }
            return null;
        }

        @Override
        public String superstructureName() {
            String name = SuperstructureMechanism.this.getName();
            if (name == null || name.isBlank()) {
                return "Superstructure";
            }
            return name;
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
        public SuperstructureMechanismsView<SP> mechanisms() {
            return SuperstructureMechanism.this.mechanisms();
        }

        @Override
        public boolean input(String key) {
            return SuperstructureMechanism.this.input().bool(key);
        }

        @Override
        public double doubleInput(String key) {
            return SuperstructureMechanism.this.input().dbl(key);
        }

        @Override
        public DoubleSupplier doubleInputSupplier(String key) {
            return SuperstructureMechanism.this.input().dblSupplier(key);
        }

        @Override
        public int intVal(String key) {
            return SuperstructureMechanism.this.input().integer(key);
        }

        @Override
        public IntSupplier intValSupplier(String key) {
            return SuperstructureMechanism.this.input().integerSupplier(key);
        }

        @Override
        public String stringVal(String key) {
            return SuperstructureMechanism.this.input().string(key);
        }

        @Override
        public Supplier<String> stringValSupplier(String key) {
            return SuperstructureMechanism.this.input().stringSupplier(key);
        }

        @Override
        public Pose2d pose2dVal(String key) {
            return SuperstructureMechanism.this.input().pose2d(key);
        }

        @Override
        public Supplier<Pose2d> pose2dValSupplier(String key) {
            return SuperstructureMechanism.this.input().pose2dSupplier(key);
        }

        @Override
        public Pose3d pose3dVal(String key) {
            return SuperstructureMechanism.this.input().pose3d(key);
        }

        @Override
        public Supplier<Pose3d> pose3dValSupplier(String key) {
            return SuperstructureMechanism.this.input().pose3dSupplier(key);
        }

        @Override
        public <T> T objectInput(String key, Class<T> type) {
            return SuperstructureMechanism.this.input().object(key, type);
        }

        @Override
        public <T> Supplier<T> objectInputSupplier(String key, Class<T> type) {
            return SuperstructureMechanism.this.input().objectSupplier(key, type);
        }

    }
}
