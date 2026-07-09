package ca.frc6390.athena.api.hardware;

/**
 * Common built-in motor kinds known to Athena.
 *
 * <p>These constants are identifiers, not vendor dependencies. For example,
 * {@link #TALON_FX} may be used in source without the CTRE adapter installed;
 * construction requires an installed backend at validation/runtime.</p>
 */
public enum MotorKinds implements MotorKind {
    /** CTRE Talon FX Phoenix 6 motor controller. */
    TALON_FX("ctre:talon-fx"),

    /** CTRE Kraken X60 Phoenix 6 motor controller. */
    KRAKEN_X60("ctre:kraken-x60"),

    /** CTRE Kraken X44 Phoenix 6 motor controller. */
    KRAKEN_X44("ctre:kraken-x44"),

    /** REV Spark MAX controlling a brushless motor. */
    SPARK_MAX_BRUSHLESS("rev:spark-max-brushless"),

    /** REV Spark MAX controlling a brushed motor. */
    SPARK_MAX_BRUSHED("rev:spark-max-brushed"),

    /** REV Spark Flex controlling a brushless motor. */
    SPARK_FLEX_BRUSHLESS("rev:spark-flex-brushless"),

    /** REV Spark Flex controlling a brushed motor. */
    SPARK_FLEX_BRUSHED("rev:spark-flex-brushed");

    private final String key;

    MotorKinds(String key) {
        this.key = key;
    }

    @Override
    public String key() {
        return key;
    }
}
