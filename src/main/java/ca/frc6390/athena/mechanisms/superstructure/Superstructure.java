package ca.frc6390.athena.mechanisms.superstructure;

import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;

import ca.frc6390.athena.mechanisms.StateMachine;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Runtime coordinator that fans macro states out to the registered mechanisms. Teams should create
 * this via {@link SuperstructureConfig#build()} and call {@link #periodic()} from their robot class.
 */
public final class Superstructure<E extends Enum<E> & SuperstructureState<C>, C extends SuperstructureContext>
        extends SubsystemBase {

    private final SuperstructureRuntime runtime;
    private final StateMachine<SuperstructureStateDef, E> stateMachine;
    private final Map<E, SuperstructureStateDef> definitions;
    private final C context;

    private SuperstructureStateDef.Session activeSession;
    private E activeState;

    Superstructure(SuperstructureConfig<E, C> config) {
        this.definitions = Objects.requireNonNull(config.definitions());
        this.context = Objects.requireNonNull(config.context());
        this.runtime = new SuperstructureRuntime(config.mechanisms());
        this.activeState = Objects.requireNonNull(config.initialState());
        this.activeSession = definitionFor(activeState).createSession();
        this.stateMachine = new StateMachine<>(activeState, this::isCurrentStateSatisfied);

        activeSession.enter(runtime);
    }

    public C context() {
        return context;
    }

    public StateMachine<SuperstructureStateDef, E> stateMachine() {
        return stateMachine;
    }

    public E activeState() {
        return activeState;
    }

    public void queue(E state) {
        queue(state, () -> true);
    }

    public void queue(E state, BooleanSupplier additionalCondition) {
        Objects.requireNonNull(additionalCondition);
        stateMachine.queueState(state,
                () -> isCurrentStateSatisfied() && additionalCondition.getAsBoolean());
    }

    public void resetQueue() {
        stateMachine.resetQueue();
    }

    public boolean isCurrentStateSatisfied() {
        return activeSession != null && activeSession.isSatisfied();
    }

    public void update() {
        if (activeSession != null) {
            activeSession.update(runtime);
            if (activeSession.shouldQueueFollowup()) {
                @SuppressWarnings("unchecked")
                E followup = (E) activeSession.followupState();
                queue(followup);
            }
        }

        stateMachine.update();

        E goalState = stateMachine.getGoalState();
        if (goalState != activeState) {
            if (activeSession != null) {
                activeSession.exit(runtime);
            }
            activeState = goalState;
            activeSession = definitionFor(activeState).createSession();
            activeSession.enter(runtime);
        }

        runtime.flushDeferred();
    }

    @Override
    public void periodic() {
        update();
    }

    private SuperstructureStateDef definitionFor(E state) {
        SuperstructureStateDef definition = definitions.get(state);
        if (definition == null) {
            throw new IllegalStateException("No definition registered for " + state.name());
        }
        return definition;
    }
}
