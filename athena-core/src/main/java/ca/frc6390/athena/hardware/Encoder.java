package ca.frc6390.athena.hardware;

/**
 * Vendor-agnostic encoder interface for the new vendordep system.
 */
public interface Encoder {
    double getPosition();

    double getVelocity();

    void setPosition(double position);

    void setInverted(boolean inverted);

    void setConversion(double conversion);

    void setOffset(double offset);

    EncoderConfig getConfig();
}
