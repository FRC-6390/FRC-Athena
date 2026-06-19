package ca.frc6390.athena.mechanisms.statespec;

public interface StateSeedProvider<E> {
    StateSeed<E> seed();
}
