package ca.frc6390.athena.runtime.validation;

/**
 * Exception thrown when a caller chooses fail-fast validation.
 */
public class AthenaValidationException extends RuntimeException {
    private static final long serialVersionUID = 1L;
    private final ValidationReport report;

    /**
     * Creates an exception from a validation report.
     *
     * @param report failing report
     */
    public AthenaValidationException(ValidationReport report) {
        super(report == null ? "Athena validation failed." : report.summary());
        this.report = report == null ? ValidationReport.ok() : report;
    }

    /**
     * Returns the validation report that caused this exception.
     *
     * @return validation report
     */
    public ValidationReport report() {
        return report;
    }
}
