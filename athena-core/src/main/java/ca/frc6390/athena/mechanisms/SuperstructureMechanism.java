package ca.frc6390.athena.mechanisms;

import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.DoubleSupplier;
import java.util.function.Function;
import java.util.function.Supplier;

import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.FlywheelMechanism.StatefulFlywheelMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.Mechanism;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;

/**
 * Composite mechanism that coordinates multiple stateful mechanisms using a superstate enum.
 * Constraints reference child mechanisms via {@link SuperstructureContext#getMechanisms()} using
 * the same mapper supplied to the config.
 */
public class SuperstructureMechanism<S extends Enum<S> & SetpointProvider<SP>, SP> extends SubsystemBase implements RobotSendableSystem, RegisterableMechanism {
    private RobotCore<?> robotCore;

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
    private final Map<String, Supplier<?>> objectInputs;
    private final Map<S, List<SuperstructureConfig.Binding<SP>>> bindings;
    private final List<SuperstructureConfig.Binding<SP>> alwaysBindings;
    private final List<SuperstructureConfig.Binding<SP>> periodicBindings;
    private final Map<S, List<SuperstructureConfig.Binding<SP>>> exitBindings;
    private final List<SuperstructureConfig.Binding<SP>> exitAlwaysBindings;
    private final SuperstructureContextImpl context;
    private S prevState;

    SuperstructureMechanism(S initialState,
                            double stateMachineDelaySeconds,
                            List<Child<SP, ?>> children,
                            Map<S, SuperstructureConfig.Constraint<S, SP>> constraints,
                            List<SuperstructureConfig.Attachment<SP, ?>> attachments,
                            Map<String, java.util.function.BooleanSupplier> inputs,
                            Map<String, DoubleSupplier> doubleInputs,
                            Map<String, Supplier<?>> objectInputs,
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
        this.objectInputs = objectInputs;
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

    /**
     * Queues a superstate, applying its constraint if present.
     */
    public void queueState(S state) {
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
        if (!Objects.equals(current, prevState)) {
            applyExitBindings(prevState);
            applySetpoints(stateMachine.getGoalStateSetpoint());
            prevState = current;
        }
        applyBindings(current);
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
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level) {
        stateMachine.shuffleboard(tab.getLayout("State Machine", BuiltInLayouts.kList), level);
        List<Mechanism> mechanisms = new ArrayList<>();
        flattenMechanisms(mechanisms, this);
        if (!mechanisms.isEmpty()) {
            ShuffleboardLayout mechanismsLayout = tab.getLayout("Mechanisms", BuiltInLayouts.kList);
            int index = 1;
            for (Mechanism mechanism : mechanisms) {
                if (mechanism == null) {
                    continue;
                }
                String name = mechanism.getName();
                if (name == null || name.isBlank()) {
                    name = "Mechanism-" + index;
                }
                ShuffleboardLayout mechanismLayout = mechanismsLayout.getLayout(name, BuiltInLayouts.kList);
                mechanism.shuffleboard(mechanismLayout, level);
                index++;
            }
        }
        return tab;
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

    @SuppressWarnings("unchecked")
    @Override
    public SuperstructureMechanism<S, SP> shuffleboard(String tab, SendableLevel level) {
        RobotSendableSystem.super.shuffleboard(tab, level);
        return this;
    }

    @SuppressWarnings("unchecked")
    @Override
    public SuperstructureMechanism<S, SP> shuffleboard(String tab) {
        RobotSendableSystem.super.shuffleboard(tab);
        return this;
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
            java.util.function.BooleanSupplier supplier = inputs.get(key);
            return supplier != null && supplier.getAsBoolean();
        }

        @Override
        public java.util.function.BooleanSupplier inputSupplier(String key) {
            java.util.function.BooleanSupplier supplier = inputs.get(key);
            if (supplier == null) {
                throw new IllegalArgumentException("No input found for key " + key);
            }
            return supplier;
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
            propagateRobotCore(robotCore, this);
        }
    }

    public RobotCore<?> getRobotCore() {
        return robotCore;
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

    public boolean input(String key) {
        java.util.function.BooleanSupplier supplier = inputs.get(key);
        return supplier != null && supplier.getAsBoolean();
    }

    public java.util.function.BooleanSupplier inputSupplier(String key) {
        java.util.function.BooleanSupplier supplier = inputs.get(key);
        if (supplier == null) {
            throw new IllegalArgumentException("No input found for key " + key);
        }
        return supplier;
    }

    public double doubleInput(String key) {
        DoubleSupplier supplier = doubleInputs.get(key);
        return supplier != null ? supplier.getAsDouble() : Double.NaN;
    }

    public DoubleSupplier doubleInputSupplier(String key) {
        DoubleSupplier supplier = doubleInputs.get(key);
        if (supplier == null) {
            throw new IllegalArgumentException("No double input found for key " + key);
        }
        return supplier;
    }

    public <T> T objectInput(String key, Class<T> type) {
        Supplier<?> supplier = objectInputs.get(key);
        if (supplier == null) {
            return null;
        }
        Object value = supplier.get();
        if (value == null) {
            return null;
        }
        if (!type.isInstance(value)) {
            throw new IllegalArgumentException("Input '" + key + "' is not of type " + type.getSimpleName());
        }
        return type.cast(value);
    }

    public <T> Supplier<T> objectInputSupplier(String key, Class<T> type) {
        Supplier<?> supplier = objectInputs.get(key);
        if (supplier == null) {
            throw new IllegalArgumentException("No object input found for key " + key);
        }
        return () -> objectInput(key, type);
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
            java.util.function.BooleanSupplier supplier = inputs.get(key);
            return supplier != null && supplier.getAsBoolean();
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
