package ca.frc6390.athena.mechanisms.statespec;

import java.util.Objects;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

/**
 * Source-visible state key for non-enum state definitions.
 */
public final class StateId implements SetpointProvider<Object>, StateSeedProvider<StateId> {
    private final StateSet owner;
    private final String name;
    private Object setpoint;
    private StateSeed<StateId> seed = StateSeed.auto();

    StateId(StateSet owner, String name) {
        this.owner = Objects.requireNonNull(owner, "owner");
        if (name == null || name.isBlank()) {
            throw new IllegalArgumentException("state name is required");
        }
        this.name = name;
    }

    public StateSet owner() {
        return owner;
    }

    public String name() {
        return name;
    }

    public StateId setpoint(double setpoint) {
        this.setpoint = Double.valueOf(setpoint);
        this.seed = StateSeed.setpoint(setpoint);
        return this;
    }

    public StateId setpoint(Object setpoint) {
        this.setpoint = setpoint;
        return this;
    }

    public StateId flow(StateDsl<StateId> dsl) {
        this.seed = StateSeed.dsl(dsl);
        return this;
    }

    public StateId manualPercent(double percent) {
        return flow(state -> state.manualPercent(percent));
    }

    public StateId until(java.util.function.Predicate<StateCtx<StateId>> predicate) {
        StateSeed<StateId> previous = seed;
        return flow(builder -> {
            apply(previous, builder);
            return builder.until(predicate);
        });
    }

    public StateId then(StateId next) {
        StateSeed<StateId> previous = seed;
        return flow(builder -> {
            apply(previous, builder);
            return builder.then(next);
        });
    }

    public StateId then(String nextName) {
        StateSeed<StateId> previous = seed;
        return flow(builder -> {
            apply(previous, builder);
            return builder.then(nextName);
        });
    }

    @Override
    public StateSeed<StateId> seed() {
        return seed;
    }

    @Override
    public Object getSetpoint() {
        return setpoint;
    }

    @Override
    public String toString() {
        return name;
    }

    private static void apply(StateSeed<StateId> seed, StateBuilder<StateId> builder) {
        if (seed == null) {
            return;
        }
        if (seed.kind() == StateSeed.Kind.SETPOINT) {
            builder.setpoint(seed.setpoint());
        } else if (seed.kind() == StateSeed.Kind.DSL && seed.dsl() != null) {
            StateBuilder<StateId> applied = seed.dsl().apply(builder);
            if (applied != null && applied != builder) {
                copy(applied, builder);
            }
        }
    }

    private static void copy(StateBuilder<StateId> source, StateBuilder<StateId> target) {
        if (source.setpoint() != null) {
            target.setpoint(source.setpoint());
        }
        if (source.manualPercent() != null) {
            target.manualPercent(source.manualPercent());
        }
        if (source.until() != null) {
            target.until(source.until());
        }
        if (source.next() != null) {
            target.then(source.next());
        } else if (source.nextName() != null) {
            target.then(source.nextName());
        }
    }
}
