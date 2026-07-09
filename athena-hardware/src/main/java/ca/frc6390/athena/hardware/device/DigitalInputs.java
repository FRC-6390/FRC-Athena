package ca.frc6390.athena.hardware.device;

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
    public static DigitalInputDevice rio(int channel) {
        return DigitalInputDevice.rio(channel);
    }

}
