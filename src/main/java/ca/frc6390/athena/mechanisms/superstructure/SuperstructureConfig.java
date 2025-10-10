package ca.frc6390.athena.mechanisms.superstructure;

import java.util.Collections;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

import ca.frc6390.athena.mechanisms.StateMachine;
import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

/**
 * Fluent builder for creating a {@link Superstructure}. Teams register each underlying mechanism's
 * {@link StateMachine}, provide an application-specific context, and declare the initial macro state.
 */
public final class SuperstructureConfig<E extends Enum<E> & SuperstructureState<C>, C extends SuperstructureContext> {

    private final Map<String, SuperstructureMechanismBinding<?>> mechanisms;
    private final Map<E, SuperstructureStateDef> definitions;
    private final E initialState;
    private final C context;

    private SuperstructureConfig(Map<String, SuperstructureMechanismBinding<?>> mechanisms,
                                 Map<E, SuperstructureStateDef> definitions,
                                 E initialState,
                                 C context) {
        this.mechanisms = mechanisms;
        this.definitions = definitions;
        this.initialState = initialState;
        this.context = context;
    }

    public Superstructure<E, C> build() {
        return new Superstructure<>(this);
    }

    Map<String, SuperstructureMechanismBinding<?>> mechanisms() {
        return mechanisms;
    }

    Map<E, SuperstructureStateDef> definitions() {
        return definitions;
    }

    E initialState() {
        return initialState;
    }

    C context() {
        return context;
    }

    public static <E extends Enum<E> & SuperstructureState<C>, C extends SuperstructureContext>
    Builder<E, C> builder(Class<E> stateClass) {
        return new Builder<>(stateClass);
    }

    public static final class Builder<E extends Enum<E> & SuperstructureState<C>, C extends SuperstructureContext> {

        private final Class<E> stateClass;
        private final Map<String, SuperstructureMechanismBinding<?>> mechanisms = new HashMap<>();
        private Supplier<C> contextSupplier = null;
        private E initialState = null;

        private Builder(Class<E> stateClass) {
            this.stateClass = Objects.requireNonNull(stateClass);
        }

        public Builder<E, C> withContext(C context) {
            this.contextSupplier = () -> Objects.requireNonNull(context);
            return this;
        }

        public Builder<E, C> withContext(Supplier<C> contextSupplier) {
            this.contextSupplier = () -> Objects.requireNonNull(contextSupplier.get());
            return this;
        }

        public Builder<E, C> initialState(E state) {
            this.initialState = Objects.requireNonNull(state);
            return this;
        }

        public <M extends Enum<M> & SetpointProvider<?>> Builder<E, C> withMechanism(
                String key, StateMachine<?, M> stateMachine) {
            mechanisms.put(key, new SuperstructureMechanismBinding<>(stateMachine));
            return this;
        }

        public SuperstructureConfig<E, C> build() {
            if (mechanisms.isEmpty()) {
                throw new IllegalStateException("At least one mechanism must be registered.");
            }

            if (initialState == null) {
                throw new IllegalStateException("Initial superstructure state must be provided.");
            }

            if (contextSupplier == null) {
                throw new IllegalStateException("A superstructure context must be provided.");
            }

            C context = contextSupplier.get();
            mechanisms.forEach(context::registerMechanism);

            Map<E, SuperstructureStateDef> definitionMap = new EnumMap<>(stateClass);
            for (E state : stateClass.getEnumConstants()) {
                SuperstructureStateDef.Builder builder = new SuperstructureStateDef.Builder();
                state.configure(builder, context);
                SuperstructureStateDef def = builder.build();
                definitionMap.put(state, def);
                SuperstructureStateRegistry.register(state, def);
            }

            return new SuperstructureConfig<>(
                    Collections.unmodifiableMap(new HashMap<>(mechanisms)),
                    Collections.unmodifiableMap(definitionMap),
                    initialState,
                    context);
        }
    }
}
