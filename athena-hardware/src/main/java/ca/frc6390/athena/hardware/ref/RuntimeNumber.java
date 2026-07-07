package ca.frc6390.athena.hardware.ref;

/**
 * Runtime handle for a numeric signal.
 */
public interface RuntimeNumber {
    /**
     * Reads the current value.
     *
     * @return current value
     */
    double get();

    /**
     * Writes the current value.
     *
     * @param value value to write
     */
    void set(double value);
}
