package ca.frc6390.athena.hardware.ref;

/**
 * Factories for boolean software signals.
 */
public final class Booleans {
    private Booleans() {
    }

    /**
     * Creates a named boolean.
     *
     * @param name signal name
     * @return boolean ref
     */
    public static BooleanRef of(String name) {
        return BooleanRef.of(name);
    }

    /**
     * Creates a named boolean with a default value.
     *
     * @param name signal name
     * @param defaultValue default value
     * @return boolean ref
     */
    public static BooleanRef of(String name, boolean defaultValue) {
        return BooleanRef.of(name, defaultValue);
    }

    /**
     * Creates an unnamed boolean with a default value.
     *
     * <p>When used as a discovered mechanism field, the field name is the ref name.</p>
     *
     * @param defaultValue default value
     * @return boolean ref
     */
    public static BooleanRef value(boolean defaultValue) {
        return BooleanRef.value(defaultValue);
    }
}
