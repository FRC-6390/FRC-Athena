package ca.frc6390.athena.hardware.ref;

/**
 * Runtime handle for a boolean signal.
 */
public interface RuntimeBoolean {
    /**
     * Reads the current value.
     *
     * @return current value
     */
    boolean get();

    /**
     * Writes the current value.
     *
     * @param value value to write
     */
    void set(boolean value);
}
