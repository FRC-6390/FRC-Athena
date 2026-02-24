package ca.frc6390.athena.mechanisms.statespec;

@FunctionalInterface
public interface StateDsl<E extends Enum<E>> {
    StateBuilder<E> apply(StateBuilder<E> builder);
}
