package ca.frc6390.athena.api.superstructure.behavior.hook;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.Objects;

import ca.frc6390.athena.api.superstructure.definition.SuperstructureHookDefinition;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureHookPhase;
import ca.frc6390.athena.api.superstructure.definition.SuperstructureTransitionHookDefinition;
import ca.frc6390.athena.core.RobotCoreHooks;
import ca.frc6390.athena.core.hooks.LifecycleHooksSectionBase;
import ca.frc6390.athena.mechanisms.statespec.StateNames;

public final class SuperstructureHooks<S, SP>
        extends LifecycleHooksSectionBase<SuperstructureHooks<S, SP>, SuperstructureHookCallback<SP>, S> {
    public record StateTransitionPair<S>(S from, S to) {
    }

    private final List<SuperstructureHookDefinition<SP>> hooks = new ArrayList<>();
    private final List<SuperstructureTransitionHookDefinition<SP>> transitions = new ArrayList<>();

    private SuperstructureHooks() {
    }

    public static <S, SP> SuperstructureHooks<S, SP> create() {
        return new SuperstructureHooks<>();
    }

    public static <S, SP> SuperstructureHooks<S, SP> from(
            List<SuperstructureHookDefinition<SP>> hooks,
            List<SuperstructureTransitionHookDefinition<SP>> transitions) {
        SuperstructureHooks<S, SP> section = create();
        if (hooks != null) {
            section.hooks.addAll(hooks);
        }
        if (transitions != null) {
            section.transitions.addAll(transitions);
        }
        return section;
    }

    @Override
    protected SuperstructureHooks<S, SP> self() {
        return this;
    }

    @Override
    protected void addPhaseBinding(RobotCoreHooks.Phase phase, SuperstructureHookCallback<SP> binding, List<S> states) {
        hooks.add(new SuperstructureHookDefinition<>(
            SuperstructureHookPhase.ROBOT_PHASE,
            java.util.Optional.of(Objects.requireNonNull(phase, "phase")),
            stateNames(states),
            Objects.requireNonNull(binding, "binding")));
    }

    @Override
    protected void addPhaseExitBinding(SuperstructureHookCallback<SP> binding, List<S> states) {
        hooks.add(new SuperstructureHookDefinition<>(
            SuperstructureHookPhase.ROBOT_PHASE_EXIT,
            java.util.Optional.empty(),
            stateNames(states),
            Objects.requireNonNull(binding, "binding")));
    }

    @SafeVarargs
    public final SuperstructureHooks<S, SP> onStatePeriodic(SuperstructureHookCallback<SP> binding, S... states) {
        requireStates("onStatePeriodic", states);
        hooks.add(new SuperstructureHookDefinition<>(
            SuperstructureHookPhase.STATE_PERIODIC,
            java.util.Optional.empty(),
            stateNames(states),
            Objects.requireNonNull(binding, "binding")));
        return this;
    }

    public SuperstructureHooks<S, SP> always(SuperstructureHookCallback<SP> binding) {
        hooks.add(new SuperstructureHookDefinition<>(
            SuperstructureHookPhase.ALWAYS,
            java.util.Optional.empty(),
            List.of(),
            Objects.requireNonNull(binding, "binding")));
        return this;
    }

    @SafeVarargs
    public final SuperstructureHooks<S, SP> onStateExit(SuperstructureHookCallback<SP> binding, S... states) {
        requireStates("onStateExit", states);
        hooks.add(new SuperstructureHookDefinition<>(
            SuperstructureHookPhase.STATE_EXIT,
            java.util.Optional.empty(),
            stateNames(states),
            Objects.requireNonNull(binding, "binding")));
        return this;
    }

    public SuperstructureHooks<S, SP> onAnyStateExit(SuperstructureHookCallback<SP> binding) {
        hooks.add(new SuperstructureHookDefinition<>(
            SuperstructureHookPhase.ANY_STATE_EXIT,
            java.util.Optional.empty(),
            List.of(),
            Objects.requireNonNull(binding, "binding")));
        return this;
    }

    @SafeVarargs
    public final SuperstructureHooks<S, SP> onStateEnter(SuperstructureHookCallback<SP> binding, S... states) {
        requireStates("onStateEnter", states);
        hooks.add(new SuperstructureHookDefinition<>(
            SuperstructureHookPhase.STATE_ENTER,
            java.util.Optional.empty(),
            stateNames(states),
            Objects.requireNonNull(binding, "binding")));
        return this;
    }

    public SuperstructureHooks<S, SP> onStateTransition(
            SuperstructureTransitionHookCallback<SP> hook,
            S from,
            S to) {
        transitions.add(new SuperstructureTransitionHookDefinition<>(
            StateNames.name(Objects.requireNonNull(from, "from")),
            StateNames.name(Objects.requireNonNull(to, "to")),
            Objects.requireNonNull(hook, "hook")));
        return this;
    }

    @SafeVarargs
    public final SuperstructureHooks<S, SP> onStateTransition(
            SuperstructureTransitionHookCallback<SP> hook,
            StateTransitionPair<S>... pairs) {
        Objects.requireNonNull(hook, "hook");
        Objects.requireNonNull(pairs, "pairs");
        for (StateTransitionPair<S> pair : pairs) {
            if (pair == null || pair.from() == null || pair.to() == null) {
                continue;
            }
            onStateTransition(hook, pair.from(), pair.to());
        }
        return this;
    }

    public SuperstructureHooks<S, SP> onRobotPeriodic(SuperstructureHookCallback<SP> binding) {
        hooks.add(new SuperstructureHookDefinition<>(
            SuperstructureHookPhase.ROBOT_PERIODIC,
            java.util.Optional.empty(),
            List.of(),
            Objects.requireNonNull(binding, "binding")));
        return this;
    }

    @SafeVarargs
    public final SuperstructureHooks<S, SP> onRobotPeriodic(SuperstructureHookCallback<SP> binding, S... states) {
        hooks.add(new SuperstructureHookDefinition<>(
            SuperstructureHookPhase.ROBOT_PERIODIC,
            java.util.Optional.empty(),
            stateNames(states),
            Objects.requireNonNull(binding, "binding")));
        return this;
    }

    public SuperstructureHooks<S, SP> merge(SuperstructureHooks<S, SP> other) {
        if (other != null) {
            hooks.addAll(other.hooks);
            transitions.addAll(other.transitions);
        }
        return this;
    }

    public List<SuperstructureHookDefinition<SP>> hookDefinitions() {
        return List.copyOf(hooks);
    }

    public List<SuperstructureTransitionHookDefinition<SP>> transitionDefinitions() {
        return List.copyOf(transitions);
    }

    private static <S> void requireStates(String method, S[] states) {
        if (states == null || states.length == 0) {
            throw new IllegalArgumentException(method + " requires at least one state");
        }
        if (Arrays.stream(states).anyMatch(Objects::isNull)) {
            throw new IllegalArgumentException(method + " does not allow null states");
        }
    }

    private static <S> List<String> stateNames(List<S> states) {
        if (states == null || states.isEmpty()) {
            return List.of();
        }
        List<String> names = new ArrayList<>(states.size());
        for (S state : states) {
            if (state != null) {
                names.add(StateNames.name(state));
            }
        }
        return List.copyOf(names);
    }

    @SafeVarargs
    private static <S> List<String> stateNames(S... states) {
        if (states == null || states.length == 0) {
            return List.of();
        }
        return stateNames(Arrays.asList(states));
    }
}
