package ca.frc6390.athena.mechanisms.statespec;

import java.util.function.Predicate;

/**
 * Minimal fluent builder used by enum DSL seeds.
 */
public class StateBuilder<E> {
    private Double setpoint;
    private Double manualPercent;
    private Predicate<StateCtx<E>> until;
    private E next;
    private String nextName;

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
        this.nextName = StateNames.name(state);
        return this;
    }

    public StateBuilder<E> then(String stateName) {
        this.next = null;
        this.nextName = stateName;
        return this;
    }

    /**
     * Separate name-based transition helper for IDEs that mis-handle the overloaded then(String).
     */
    public StateBuilder<E> thenNamed(String stateName) {
        return then(stateName);
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

    public String nextName() {
        return nextName;
    }
}
