package ca.frc6390.athena.mechanisms.statespec;

public interface StateSeedProvider<E extends Enum<E>> {
    StateSeed<E> seed();
}
