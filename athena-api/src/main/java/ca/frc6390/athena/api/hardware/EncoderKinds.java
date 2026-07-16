package ca.frc6390.athena.api.hardware;

/**
 * Common built-in encoder kinds known to Athena.
 */
public enum EncoderKinds implements EncoderKind {
    /** CTRE CANcoder absolute encoder. */
    CANCODER("ctre:cancoder"),

    /** REV through-bore V1 absolute PWM encoder (legacy key retained for compatibility). */
    REV_THROUGH_BORE("rev:through-bore"),

    /** REV through-bore V2 absolute PWM encoder. */
    REV_THROUGH_BORE_V2("rev:through-bore-v2"),

    /** REV through-bore incremental quadrature encoder. */
    REV_THROUGH_BORE_QUADRATURE("rev:through-bore-quadrature"),

    /** Encoder integrated into the selected motor controller. */
    INTEGRATED_MOTOR("athena:integrated-motor");

    private final String key;

    EncoderKinds(String key) {
        this.key = key;
    }

    @Override
    public String key() {
        return key;
    }
}
