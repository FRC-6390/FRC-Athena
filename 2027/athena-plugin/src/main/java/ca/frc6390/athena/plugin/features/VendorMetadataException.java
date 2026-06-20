package ca.frc6390.athena.plugin.features;

/**
 * Raised when Athena vendor metadata resources are malformed or conflicting.
 */
public final class VendorMetadataException extends RuntimeException {
    private static final long serialVersionUID = 1L;

    /**
     * Creates an exception.
     *
     * @param message diagnostic message
     */
    public VendorMetadataException(String message) {
        super(message);
    }

    /**
     * Creates an exception with a cause.
     *
     * @param message diagnostic message
     * @param cause failure cause
     */
    public VendorMetadataException(String message, Throwable cause) {
        super(message, cause);
    }
}
