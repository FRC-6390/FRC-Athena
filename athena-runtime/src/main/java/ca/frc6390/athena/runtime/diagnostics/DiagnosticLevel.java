package ca.frc6390.athena.runtime.diagnostics;

/**
 * Severity for diagnostic events.
 */
public enum DiagnosticLevel {
    /** Informational event. */
    INFO,

    /** Warning event that should be visible but is not fatal by itself. */
    WARN,

    /** Error event that indicates a failing subsystem or configuration. */
    ERROR
}
