package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.hardware.encoder.Encoder;

/**
 * Resolved named encoder source available to a mechanism at runtime.
 */
public record MechanismEncoderSource(
        String name,
        Encoder device,
        MechanismEncoderUnit unit,
        double wrapsEvery,
        boolean positionSupported,
        boolean velocitySupported,
        boolean absoluteSupported) {

    public boolean supportsPosition() {
        return device != null && positionSupported;
    }

    public boolean supportsVelocity() {
        return device != null && velocitySupported;
    }

    public boolean supportsAbsolute() {
        return device != null && absoluteSupported;
    }

    public boolean supportsWrappedAbsolute() {
        return supportsAbsolute() && Double.isFinite(wrapsEvery) && wrapsEvery > 0.0;
    }
}
