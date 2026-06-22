package ca.frc6390.athena.auto;

/**
 * Raised when an autonomous source key is requested without a registered
 * provider.
 */
public final class MissingAutoSourceException extends IllegalStateException {
    private static final long serialVersionUID = 1L;

    /**
     * Creates an exception for a missing source key.
     *
     * @param key missing source key
     */
    public MissingAutoSourceException(String key) {
        super("No auto source registered for '" + key
                + "'. Install the matching athena-* auto module or register an AutoSource explicitly.");
    }
}
