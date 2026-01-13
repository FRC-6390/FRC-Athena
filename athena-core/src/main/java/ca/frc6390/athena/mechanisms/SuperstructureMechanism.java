package ca.frc6390.athena.mechanisms;

import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Function;
import java.util.function.Predicate;

import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.core.RobotSendableSystem.SendableLevel;
import ca.frc6390.athena.mechanisms.ArmMechanism.StatefulArmMechanism;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;
import ca.frc6390.athena.mechanisms.Mechanism;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.ArrayList;

/**
 * Composite mechanism that coordinates multiple stateful mechanisms using a superstate enum.
 * Guards reference child mechanisms via {@link SuperstructureContext#getMechanisms()} using
 * the same mapper supplied to the config.
 */
public class SuperstructureMechanism<S extends Enum<S> & SetpointProvider<SP>, SP> extends SubsystemBase implements RobotSendableSystem, RegisterableMechanism {

    static final class Child<SP, E extends Enum<E> & SetpointProvider<?>> {
        final Mechanism mechanism;
        final SuperstructureMechanism<?, ?> superstructure;
        final StateMachine<?, ?> stateMachine;
        final Function<SP, E> mapper;
        final Class<?> stateType;

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
    private final Map<S, Predicate<SuperstructureContext<SP>>> guards;
    private final List<SuperstructureConfig.Attachment<SP, ?>> attachments;
    private final Map<String, java.util.function.BooleanSupplier> inputs;
    private final SuperstructureContextImpl context;
    private S prevState;

    SuperstructureMechanism(S initialState,
                            double stateMachineDelaySeconds,
                            List<Child<SP, ?>> children,
                            Map<S, Predicate<SuperstructureContext<SP>>> guards,
                            List<SuperstructureConfig.Attachment<SP, ?>> attachments,
                            Map<String, java.util.function.BooleanSupplier> inputs) {
        this.children = children;
        this.guards = guards;
        this.attachments = attachments;
        this.inputs = inputs;
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
     * Queues a superstate, applying its guard if present.
     */
    public void queueState(S state) {
        Predicate<SuperstructureContext<SP>> guard = guards.getOrDefault(state, ctx -> true);
        stateMachine.queueState(state, () -> guard.test(context));
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
        stateMachine.update();
        applyAttachments();
        S current = stateMachine.getGoalState();
        if (!Objects.equals(current, prevState)) {
            applySetpoints(stateMachine.getGoalStateSetpoint());
            prevState = current;
        }
    }

    private void applyAttachments() {
        for (SuperstructureConfig.Attachment<SP, ?> attachment : attachments) {
            setAttachmentPose(attachment);
        }
    }

    @SuppressWarnings("unchecked")
    private <E extends Enum<E> & SetpointProvider<Double>> void setAttachmentPose(SuperstructureConfig.Attachment<SP, E> attachment) {
        try {
            StatefulMechanism<E> child = context.mechanism(attachment.childMapper);
            var pose = attachment.poseSupplier.apply(context);
            child.setVisualizationRootOverride(pose);
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
            Mechanism mech = context.mechanism(mapper);
            if (mech instanceof StatefulArmMechanism<?>) {
                return (StatefulArmMechanism<E>) mech;
            }
            throw new IllegalArgumentException("Mapper does not resolve to an arm mechanism");
        }

        @SuppressWarnings("unchecked")
        @Override
        public <E extends Enum<E> & SetpointProvider<Double>> ElevatorMechanism.StatefulElevatorMechanism<E> elevator(Function<SP, E> mapper) {
            Mechanism mech = context.mechanism(mapper);
            if (mech instanceof ElevatorMechanism.StatefulElevatorMechanism<?>) {
                return (ElevatorMechanism.StatefulElevatorMechanism<E>) mech;
            }
            throw new IllegalArgumentException("Mapper does not resolve to an elevator mechanism");
        }

        @SuppressWarnings("unchecked")
        @Override
        public <E extends Enum<E> & SetpointProvider<Double>> TurretMechanism.StatefulTurretMechanism<E> turret(Function<SP, E> mapper) {
            Mechanism mech = context.mechanism(mapper);
            if (mech instanceof TurretMechanism.StatefulTurretMechanism<?>) {
                return (TurretMechanism.StatefulTurretMechanism<E>) mech;
            }
            throw new IllegalArgumentException("Mapper does not resolve to a turret mechanism");
        }

        @SuppressWarnings("unchecked")
        @Override
        public <E extends Enum<E> & SetpointProvider<Double>> StatefulMechanism<E> generic(Function<SP, E> mapper) {
            return context.mechanism(mapper);
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

    private final class SuperstructureContextImpl implements SuperstructureContext<SP> {
        @Override
        public SP setpoint() {
            return stateMachine.getGoalStateSetpoint();
        }

        @Override
        @SuppressWarnings("unchecked")
        public <E extends Enum<E> & SetpointProvider<Double>> StatefulMechanism<E> mechanism(Function<SP, E> mapper) {
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
                    return (StatefulMechanism<E>) child.mechanism;
                }
                if (desiredType != null && desiredType.equals(child.stateType)) {
                    return (StatefulMechanism<E>) child.mechanism;
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

    }
}
