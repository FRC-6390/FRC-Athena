package ca.frc6390.athena.mechanisms.statespec;

import java.util.function.Predicate;

/**
 * Minimal fluent builder used by enum DSL seeds.
 */
public class StateBuilder<E extends Enum<E>> {
    private Double setpoint;
    private Double manualPercent;
    private Predicate<StateCtx<E>> until;
    private E next;

    public StateBuilder<E> setpoint(double value) {
        this.setpoint = value;
        return this;
    }

    public StateBuilder<E> manualPercent(double value) {
        this.manualPercent = value;
        return this;
    }

    public StateBuilder<E> until(Predicate<StateCtx<E>> predicate) {
        this.until = predicate;
        return this;
    }

    public StateBuilder<E> then(E state) {
        this.next = state;
        return this;
    }

    public Double setpoint() {
        return setpoint;
    }

    public Double manualPercent() {
        return manualPercent;
    }

    public Predicate<StateCtx<E>> until() {
        return until;
    }

    public E next() {
        return next;
    }
}
