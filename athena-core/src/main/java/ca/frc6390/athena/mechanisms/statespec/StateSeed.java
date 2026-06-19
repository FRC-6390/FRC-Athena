package ca.frc6390.athena.mechanisms.statespec;

import java.util.Objects;

public final class StateSeed<E> {
    public enum Kind {
        AUTO,
        SETPOINT,
        DSL
    }

    private final Kind kind;
    private final double setpoint;
    private final StateDsl<E> dsl;

    private StateSeed(Kind kind, double setpoint, StateDsl<E> dsl) {
        this.kind = Objects.requireNonNull(kind, "kind");
        this.setpoint = setpoint;
        this.dsl = dsl;
    }

    public static <E> StateSeed<E> auto() {
        return new StateSeed<>(Kind.AUTO, Double.NaN, null);
    }

    public static <E> StateSeed<E> setpoint(double setpoint) {
        return new StateSeed<>(Kind.SETPOINT, setpoint, null);
    }

    public static <E> StateSeed<E> dsl(StateDsl<E> dsl) {
        return new StateSeed<>(Kind.DSL, Double.NaN, Objects.requireNonNull(dsl, "dsl"));
    }

    public Kind kind() {
        return kind;
    }

    public double setpoint() {
        return setpoint;
    }

    public StateDsl<E> dsl() {
        return dsl;
    }
}
