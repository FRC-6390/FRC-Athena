package ca.frc6390.athena.networktables;

/**
 * Active annotation binding handle.
 */
public interface AthenaNTBinding extends AutoCloseable {
    @Override
    void close();
}
