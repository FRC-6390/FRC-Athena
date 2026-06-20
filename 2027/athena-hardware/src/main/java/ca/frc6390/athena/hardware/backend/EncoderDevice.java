package ca.frc6390.athena.hardware.backend;

import ca.frc6390.athena.hardware.encoder.EncoderSpec;

/**
 * Runtime encoder created by a backend.
 */
public interface EncoderDevice {
    /**
     * Returns the normalized spec used to create this device.
     *
     * @return encoder spec
     */
    EncoderSpec spec();

    /**
     * Returns relative position in rotations.
     *
     * @return relative position rotations
     */
    default double positionRotations() {
        throw new UnsupportedOperationException("Position is not implemented by " + spec().path());
    }

    /**
     * Returns absolute position in rotations.
     *
     * @return absolute position rotations
     */
    default double absolutePositionRotations() {
        throw new UnsupportedOperationException("Absolute position is not implemented by " + spec().path());
    }

    /**
     * Returns velocity in rotations per second.
     *
     * @return velocity rotations per second
     */
    default double velocityRotationsPerSecond() {
        throw new UnsupportedOperationException("Velocity is not implemented by " + spec().path());
    }
}
