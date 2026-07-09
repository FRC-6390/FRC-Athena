package ca.frc6390.athena.mechanism.core;

/**
 * Internal mutable digital input used by mechanism simulation bindings.
 */
final class SimDigitalInput {
    private boolean value;

    boolean get() {
        return value;
    }

    void set(boolean value) {
        this.value = value;
    }
}
