package ca.frc6390.athena.runtime.validation;

import java.io.Serializable;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Optional;
import java.util.stream.Collectors;

/**
 * Immutable validation result.
 */
public final class ValidationReport implements Serializable {
    private static final long serialVersionUID = 1L;

    private static final ValidationReport OK = new ValidationReport(List.of());

    private final ArrayList<AthenaError> errors;

    private ValidationReport(List<AthenaError> errors) {
        this.errors = new ArrayList<>(errors);
    }

    /**
     * Returns an empty successful report.
     *
     * @return success report
     */
    public static ValidationReport ok() {
        return OK;
    }

    /**
     * Creates a builder for accumulating validation errors.
     *
     * @return report builder
     */
    public static Builder builder() {
        return new Builder();
    }

    /**
     * Returns true when validation found at least one error.
     *
     * @return true if errors are present
     */
    public boolean hasErrors() {
        return !errors.isEmpty();
    }

    /**
     * Returns validation errors.
     *
     * @return immutable error list
     */
    public List<AthenaError> errors() {
        return List.copyOf(errors);
    }

    /**
     * Returns the first validation error, if present.
     *
     * @return optional first error
     */
    public Optional<AthenaError> firstError() {
        return errors.stream().findFirst();
    }

    /**
     * Returns errors matching a stable code.
     *
     * @param code error code
     * @return matching errors
     */
    public List<AthenaError> errorsWithCode(String code) {
        return errors.stream()
                .filter(error -> error.code().equals(code))
                .toList();
    }

    /**
     * Throws an {@link AthenaValidationException} if this report contains
     * errors.
     */
    public void assertValid() {
        if (hasErrors()) {
            throw new AthenaValidationException(this);
        }
    }

    /**
     * Formats this report as a compact multi-line summary.
     *
     * @return summary text
     */
    public String summary() {
        if (!hasErrors()) {
            return "Validation OK";
        }
        return errors.stream()
                .map(error -> error.path() + " [" + error.code() + "]: " + error.message())
                .collect(Collectors.joining(System.lineSeparator()));
    }

    /**
     * Mutable builder for a report.
     */
    public static final class Builder {
        private final List<AthenaError> errors = new ArrayList<>();

        /**
         * Adds an error.
         *
         * @param code stable error code
         * @param path dotted path
         * @param message explanation
         * @return this builder
         */
        public Builder error(String code, String path, String message) {
            errors.add(new AthenaError(code, path, message));
            return this;
        }

        /**
         * Adds all errors from another report.
         *
         * @param report report to merge
         * @return this builder
         */
        public Builder addAll(ValidationReport report) {
            errors.addAll(report == null ? Collections.emptyList() : report.errors());
            return this;
        }

        /**
         * Adds an error object.
         *
         * @param error error to add
         * @return this builder
         */
        public Builder error(AthenaError error) {
            if (error != null) {
                errors.add(error);
            }
            return this;
        }

        /**
         * Builds the immutable report.
         *
         * @return validation report
         */
        public ValidationReport build() {
            return errors.isEmpty() ? OK : new ValidationReport(errors);
        }
    }
}
