package ca.frc6390.athena.api.mechanism.validation;

import java.util.Objects;

public record MechanismValidationIssue(
    String code,
    ValidationSeverity severity,
    String message
) {
    public MechanismValidationIssue {
        code = Objects.requireNonNull(code, "code");
        severity = Objects.requireNonNull(severity, "severity");
        message = Objects.requireNonNull(message, "message");
    }
}
