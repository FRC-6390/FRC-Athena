package ca.frc6390.athena.runtime.validation;

import java.io.Serializable;
import java.util.Objects;

/**
 * A single validation failure or warning emitted by Athena.
 *
 * @param code stable machine-readable error code
 * @param path dotted path to the failing declaration
 * @param message human-readable explanation
 */
public record AthenaError(String code, String path, String message) implements Serializable {
    private static final long serialVersionUID = 1L;

    public AthenaError {
        Objects.requireNonNull(code, "code");
        Objects.requireNonNull(path, "path");
        Objects.requireNonNull(message, "message");
    }
}
