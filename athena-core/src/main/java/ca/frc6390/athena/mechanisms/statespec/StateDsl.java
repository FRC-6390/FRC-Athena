package ca.frc6390.athena.mechanisms.statespec;

@FunctionalInterface
public interface StateDsl<E> {
    StateBuilder<E> apply(StateBuilder<E> builder);
}
