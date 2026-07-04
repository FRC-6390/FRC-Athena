package ca.frc6390.athena.hardware.ref;

/**
 * Factories for digital inputs.
 */
public final class DigitalInputs {
    private DigitalInputs() {
    }

    /**
     * Creates a roboRIO digital input.
     *
     * @param channel DIO channel
     * @return input ref
     */
    public static DigitalInputRef rio(int channel) {
        return DigitalInputRef.rio(channel);
    }

    /**
     * Creates a named roboRIO digital input.
     *
     * @param name input name
     * @param channel DIO channel
     * @return input ref
     */
    public static DigitalInputRef rio(String name, int channel) {
        return DigitalInputRef.named(name, channel);
    }
}
