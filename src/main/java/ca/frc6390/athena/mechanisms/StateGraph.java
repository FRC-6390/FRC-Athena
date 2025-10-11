package ca.frc6390.athena.mechanisms;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.BooleanSupplier;

/**
 * Declarative mapping of legal state transitions for {@link StateMachine}. Teams can describe the
 * required intermediate states between two endpoints so that higher-level code only needs to
 * request the final state and the machine automatically expands the full path.
 *
 * @param <E> enum type backing the state machine
 */
public final class StateGraph<E extends Enum<E>> {

    private final Class<E> enumClass;
    private final Map<E, Map<E, List<E>>> transitions;
    private final Map<E, Map<E, BooleanSupplier>> guards;

    private StateGraph(Class<E> enumClass) {
        this.enumClass = Objects.requireNonNull(enumClass, "enumClass");
        this.transitions = new EnumMap<>(enumClass);
        this.guards = new EnumMap<>(enumClass);
    }

    public static <E extends Enum<E>> StateGraph<E> create(Class<E> enumClass) {
        return new StateGraph<>(enumClass);
    }

    @SafeVarargs
    public final StateGraph<E> path(E... states) {
        Objects.requireNonNull(states, "states");
        if (states.length < 2) {
            throw new IllegalArgumentException("A path requires at least two states.");
        }
        for (E state : states) {
            Objects.requireNonNull(state, "Path entries cannot be null.");
        }

        for (int start = 0; start < states.length - 1; start++) {
            for (int end = start + 1; end < states.length; end++) {
                List<E> subPath = new ArrayList<>(end - start);
                for (int idx = start + 1; idx <= end; idx++) {
                    subPath.add(states[idx]);
                }
                register(states[start], states[end], subPath);
            }
        }

        return this;
    }

    public boolean hasPath(E from, E to) {
        Objects.requireNonNull(from, "from");
        Objects.requireNonNull(to, "to");
        return from.equals(to)
                || transitions.getOrDefault(from, Map.of()).containsKey(to);
    }

    public List<E> expand(E from, E to) {
        Objects.requireNonNull(from, "from");
        Objects.requireNonNull(to, "to");
        if (from.equals(to)) {
            return List.of();
        }
        Map<E, List<E>> edges = transitions.get(from);
        if (edges == null) {
            return List.of(to);
        }
        return edges.getOrDefault(to, List.of(to));
    }

    public StateGraph<E> guard(E from, E to, BooleanSupplier guard) {
        Objects.requireNonNull(from, "from");
        Objects.requireNonNull(to, "to");
        Objects.requireNonNull(guard, "guard");
        guards.computeIfAbsent(from, k -> new EnumMap<>(enumClass)).put(to, guard);
        return this;
    }

    @SafeVarargs
    public final StateGraph<E> guardPath(BooleanSupplier guard, E... states) {
        Objects.requireNonNull(guard, "guard");
        Objects.requireNonNull(states, "states");
        if (states.length < 2) {
            throw new IllegalArgumentException("A guarded path requires at least two states.");
        }
        for (int i = 0; i < states.length - 1; i++) {
            E from = Objects.requireNonNull(states[i], "Path entries cannot be null.");
            E to = Objects.requireNonNull(states[i + 1], "Path entries cannot be null.");
            guard(from, to, guard);
        }
        return this;
    }

    BooleanSupplier guardFor(E from, E to) {
        return guards
                .getOrDefault(from, Map.of())
                .getOrDefault(to, Guards.always());
    }

    private void register(E from, E to, List<E> path) {
        Map<E, List<E>> edges = transitions.computeIfAbsent(from, k -> new EnumMap<>(enumClass));
        List<E> copy = List.copyOf(path);
        List<E> existing = edges.put(to, copy);
        if (existing != null && !existing.equals(copy)) {
            throw new IllegalStateException(
                    "Conflicting path definition from " + from.name() + " to " + to.name());
        }
    }

    public static final class Guards {
        private Guards() {}

        public static BooleanSupplier always() {
            return () -> true;
        }

        public static BooleanSupplier not(BooleanSupplier condition) {
            Objects.requireNonNull(condition, "condition");
            return () -> !condition.getAsBoolean();
        }

        public static BooleanSupplier allOf(BooleanSupplier... conditions) {
            Objects.requireNonNull(conditions, "conditions");
            if (conditions.length == 0) {
                return always();
            }
            Arrays.stream(conditions).forEach(cond -> Objects.requireNonNull(cond, "conditions must not contain null"));
            return () -> Arrays.stream(conditions).allMatch(BooleanSupplier::getAsBoolean);
        }

        public static BooleanSupplier anyOf(BooleanSupplier... conditions) {
            Objects.requireNonNull(conditions, "conditions");
            if (conditions.length == 0) {
                return always();
            }
            Arrays.stream(conditions).forEach(cond -> Objects.requireNonNull(cond, "conditions must not contain null"));
            return () -> Arrays.stream(conditions).anyMatch(BooleanSupplier::getAsBoolean);
        }
    }
}
