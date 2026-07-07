package ca.frc6390.athena.hardware.ref;

/**
 * Runtime handle for an encoder.
 */
public interface RuntimeEncoder {
    /**
     * Reads relative position.
     *
     * @return position in mechanism units
     */
    double position();

    /**
     * Reads absolute position.
     *
     * @return absolute position in mechanism units
     */
    double absolutePosition();

    /**
     * Reads velocity.
     *
     * @return velocity in mechanism units per second
     */
    double velocity();

    /**
     * Sets relative position to zero.
     */
    default void zero() {
        set(0.0);
    }

    /**
     * Sets relative position.
     *
     * @param position position in mechanism units
     */
    void set(double position);

    /**
     * Sets simulated velocity when the encoder is backed by a software model.
     *
     * @param velocity velocity in mechanism units per second
     */
    default void setVelocity(double velocity) {
    }

    /**
     * Sets relative position.
     *
     * @param position position in mechanism units
     */
    default void setPosition(double position) {
        set(position);
    }

    /**
     * Syncs this encoder from another encoder source.
     *
     * @param source source encoder
     */
    default void syncTo(RuntimeEncoder source) {
        set(source.absolutePosition());
    }
}
