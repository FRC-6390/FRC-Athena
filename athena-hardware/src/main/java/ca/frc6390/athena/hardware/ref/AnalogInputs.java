package ca.frc6390.athena.hardware.ref;

/**
 * Factories for analog inputs.
 */
public final class AnalogInputs {
    private AnalogInputs() {
    }

    /**
     * Creates a roboRIO analog input.
     *
     * @param channel analog channel
     * @return input ref
     */
    public static AnalogInputRef rio(int channel) {
        return AnalogInputRef.rio(channel);
    }

    /**
     * Creates a named roboRIO analog input.
     *
     * @param name input name
     * @param channel analog channel
     * @return input ref
     */
    public static AnalogInputRef rio(String name, int channel) {
        return AnalogInputRef.named(name, channel);
    }
}
