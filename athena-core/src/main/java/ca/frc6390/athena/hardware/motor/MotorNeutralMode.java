package ca.frc6390.athena.hardware.motor;

/**
 * Vendor-agnostic motor neutral behavior.
 */
public enum MotorNeutralMode {
    Coast,
    Brake;

    public static MotorNeutralMode fromBoolean(boolean brake) {
        return brake ? Brake : Coast;
    }
}
