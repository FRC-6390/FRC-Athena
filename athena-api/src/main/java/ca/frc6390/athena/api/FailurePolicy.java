package ca.frc6390.athena.api;

/** Response Athena applies when a declared runtime dependency fails. */
public enum FailurePolicy {
    /** Propagates the failure and stops normal robot-code execution. */
    PANIC,
    /** Reports the failure and suppresses the mechanism that owns the declaration. */
    DISABLE_MECHANISM,
    /** Reports the failure and suppresses only the failed declaration. */
    DISABLE_DEVICE,
    /** Reports the failure, retains the last cached input, and retries next cycle. */
    WARN
}
