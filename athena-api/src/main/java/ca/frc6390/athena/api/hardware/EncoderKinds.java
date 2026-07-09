package ca.frc6390.athena.api.hardware;

/**
 * Common built-in encoder kinds known to Athena.
 */
public enum EncoderKinds implements EncoderKind {
    /** CTRE CANcoder absolute encoder. */
    CANCODER("ctre:cancoder"),

    /** REV through-bore absolute encoder. */
    REV_THROUGH_BORE("rev:through-bore"),

    /** Encoder integrated into the selected motor controller. */
    INTEGRATED_MOTOR("athena:integrated-motor"),

    /** Simulation-only encoder backend. */
    SIM("sim:encoder");

    private final String key;

    EncoderKinds(String key) {
        this.key = key;
    }

    @Override
    public String key() {
        return key;
    }
}
