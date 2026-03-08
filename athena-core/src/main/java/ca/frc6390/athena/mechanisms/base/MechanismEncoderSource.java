package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.hardware.encoder.Encoder;

/**
 * Resolved named encoder source available to a mechanism at runtime.
 */
public record MechanismEncoderSource(
        String name,
        Encoder device,
        MechanismEncoderUnit unit,
        double wrapsEvery) {

    public boolean supportsAbsolute() {
        return device != null;
    }

    public boolean supportsWrappedAbsolute() {
        return device != null && Double.isFinite(wrapsEvery) && wrapsEvery > 0.0;
    }
}
