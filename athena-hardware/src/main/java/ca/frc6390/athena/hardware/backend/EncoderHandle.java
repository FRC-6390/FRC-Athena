package ca.frc6390.athena.hardware.backend;

import ca.frc6390.athena.hardware.device.EncoderDevice;

/**
 * Runtime encoder created by a backend.
 *
 * <p>Backend SPI: vendors implement these primitive methods. Robot code never calls them; it reads the
 * typed device accessors, or {@code raw()} for Athena's own per-cycle code. Not for robot code.</p>
 */
public interface EncoderHandle {
    /**
     * Returns the declaration used to create this handle.
     *
     * @return encoder declaration
     */
    EncoderDevice device();

    /**
     * Activates runtime configuration after the graph creates and caches the handle.
     */
    default void activate() {
        // default no-op
    }

    /**
     * Refreshes runtime input snapshots before graph consumers read this handle.
     */
    default void refreshInputs() {
        // default no-op
    }

    /**
     * Refreshes runtime input snapshots for one refresh pass. Handles that share a vendor batched read use the
     * pass token to read the shared bus once per pass; a token must not repeat across passes. The default ignores
     * the token.
     *
     * @param refreshPass token unique to this refresh pass
     */
    default void refreshInputs(long refreshPass) {
        refreshInputs();
    }

    /**
     * Returns relative position in rotations.
     *
     * @return relative position rotations
     */
    default double positionRotations() {
        throw new UnsupportedOperationException("Position is not implemented by " + device().defaultName());
    }

    /**
     * Returns absolute position in rotations.
     *
     * @return absolute position rotations
     */
    default double absolutePositionRotations() {
        throw new UnsupportedOperationException("Absolute position is not implemented by " + device().defaultName());
    }

    /**
     * Returns velocity in rotations per second.
     *
     * @return velocity rotations per second
     */
    default double velocityRotationsPerSecond() {
        throw new UnsupportedOperationException("Velocity is not implemented by " + device().defaultName());
    }

    /**
     * Sets relative position in rotations.
     *
     * @param rotations relative position rotations
     */
    default void setPositionRotations(double rotations) {
        throw new UnsupportedOperationException("Setting position is not implemented by " + device().defaultName());
    }

    /**
     * Returns whether this handle can set relative position natively.
     *
     * @return true when native position setting is supported
     */
    default boolean supportsPositionSetting() {
        return false;
    }
}
