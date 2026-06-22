package ca.frc6390.athena.dashboard;

/**
 * Runtime failure raised by dashboard transport threads.
 */
public final class DashboardTransportException extends RuntimeException {
    private static final long serialVersionUID = 1L;

    /**
     * Creates a transport exception.
     *
     * @param message failure description
     * @param cause underlying failure
     */
    public DashboardTransportException(String message, Throwable cause) {
        super(message, cause);
    }
}
