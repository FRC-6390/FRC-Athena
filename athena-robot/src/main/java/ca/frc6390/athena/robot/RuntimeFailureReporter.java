package ca.frc6390.athena.robot;

/** Host-specific destination for recoverable Athena runtime failures. */
public interface RuntimeFailureReporter {
    void error(String message, Throwable cause);

    void warning(String message, Throwable cause);

    /** Queues reports to a bounded worker; custom destinations are synchronous unless wrapped here. */
    static AsyncRuntimeFailureReporter async(RuntimeFailureReporter destination) {
        return destination instanceof AsyncRuntimeFailureReporter async ? async
                : new AsyncRuntimeFailureReporter(destination);
    }

    static RuntimeFailureReporter stderr() { return async(StandardError.INSTANCE); }

    final class StandardError {
        private StandardError() { }
        static final RuntimeFailureReporter INSTANCE = new RuntimeFailureReporter() {
            @Override public void error(String message, Throwable cause) {
                System.err.println("ERROR: " + message);
                if (cause != null) cause.printStackTrace(System.err);
            }

            @Override public void warning(String message, Throwable cause) {
                System.err.println("WARNING: " + message);
                if (cause != null) cause.printStackTrace(System.err);
            }
        };
    }
}
