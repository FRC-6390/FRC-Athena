package ca.frc6390.athena.hardware.ref;

/**
 * Runtime access used by ref actions.
 */
public interface ActionContext {
    /**
     * Resolves an encoder declaration to its runtime handle.
     *
     * @param ref encoder declaration
     * @return runtime encoder
     */
    default RuntimeEncoder encoder(EncoderRef ref) {
        throw new UnsupportedOperationException("Runtime encoder access is not available.");
    }

    /**
     * Resolves a motor declaration to its runtime handle.
     *
     * @param ref motor declaration
     * @return runtime motor
     */
    default RuntimeMotor motor(MotorRef ref) {
        throw new UnsupportedOperationException("Runtime motor access is not available.");
    }

    /**
     * Resolves a boolean signal declaration to its runtime handle.
     *
     * @param ref boolean declaration
     * @return runtime boolean
     */
    default RuntimeBoolean bool(BooleanRef ref) {
        throw new UnsupportedOperationException("Runtime boolean access is not available.");
    }

    /**
     * Resolves a numeric signal declaration to its runtime handle.
     *
     * @param ref number declaration
     * @return runtime number
     */
    default RuntimeNumber number(NumberRef ref) {
        throw new UnsupportedOperationException("Runtime number access is not available.");
    }

    /**
     * Empty context for tests that only execute pure runnable actions.
     *
     * @return empty action context
     */
    static ActionContext empty() {
        return new ActionContext() {
        };
    }
}
