package ca.frc6390.athena.hardware.ref;

/**
 * Factories for numeric software signals.
 */
public final class Numbers {
    private Numbers() {
    }

    /**
     * Creates a named number.
     *
     * @param name signal name
     * @return number ref
     */
    public static NumberRef of(String name) {
        return NumberRef.of(name);
    }

    /**
     * Creates a named number with a default value.
     *
     * @param name signal name
     * @param defaultValue default value
     * @return number ref
     */
    public static NumberRef of(String name, double defaultValue) {
        return NumberRef.of(name, defaultValue);
    }
}
