package ca.frc6390.athena.robot;

/** Host-specific destination for recoverable Athena runtime failures. */
public interface RuntimeFailureReporter {
    void error(String message, Throwable cause);

    void warning(String message, Throwable cause);

    static RuntimeFailureReporter stderr() {
        return new RuntimeFailureReporter() {
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
