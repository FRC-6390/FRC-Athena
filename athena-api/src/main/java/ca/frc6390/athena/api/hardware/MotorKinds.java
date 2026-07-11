package ca.frc6390.athena.api.hardware;

/**
 * Common physical FRC motor models with their usual controller pairing.
 *
 * <p>The physical model lets simulation select accurate motor constants. Use
 * {@link MotorKind#controlledBy(MotorControllerKind)} when a motor is attached to a
 * controller other than its default.</p>
 */
public enum MotorKinds implements MotorKind {
    FALCON_500("falcon-500", MotorControllerKinds.TALON_FX, MotorTechnology.BRUSHLESS),
    KRAKEN_X60("kraken-x60", MotorControllerKinds.TALON_FX, MotorTechnology.BRUSHLESS),
    KRAKEN_X44("kraken-x44", MotorControllerKinds.TALON_FX, MotorTechnology.BRUSHLESS),
    MINION("minion", MotorControllerKinds.TALON_FXS, MotorTechnology.BRUSHLESS),

    NEO("neo", MotorControllerKinds.SPARK_MAX, MotorTechnology.BRUSHLESS),
    NEO_550("neo-550", MotorControllerKinds.SPARK_MAX, MotorTechnology.BRUSHLESS),
    NEO_VORTEX("neo-vortex", MotorControllerKinds.SPARK_FLEX, MotorTechnology.BRUSHLESS),

    CIM("cim", MotorControllerKinds.SPARK_MAX, MotorTechnology.BRUSHED),
    MINI_CIM("mini-cim", MotorControllerKinds.SPARK_MAX, MotorTechnology.BRUSHED),
    BAG("bag", MotorControllerKinds.SPARK_MAX, MotorTechnology.BRUSHED),
    VEX_775_PRO("vex-775-pro", MotorControllerKinds.SPARK_MAX, MotorTechnology.BRUSHED),
    ANDYMARK_9015("andymark-9015", MotorControllerKinds.SPARK_MAX, MotorTechnology.BRUSHED),
    ANDYMARK_RS775_125("andymark-rs775-125", MotorControllerKinds.SPARK_MAX, MotorTechnology.BRUSHED),
    BANEBOTS_RS550("banebots-rs550", MotorControllerKinds.SPARK_MAX, MotorTechnology.BRUSHED),
    BANEBOTS_RS775("banebots-rs775", MotorControllerKinds.SPARK_MAX, MotorTechnology.BRUSHED);

    private final String motorKey;
    private final MotorControllerKind controllerKind;
    private final MotorTechnology technology;

    MotorKinds(String motorKey, MotorControllerKind controllerKind, MotorTechnology technology) {
        this.motorKey = motorKey;
        this.controllerKind = controllerKind;
        this.technology = technology;
    }

    @Override
    public String key() {
        return controllerKind.key() + "/" + motorKey;
    }

    @Override
    public String motorKey() {
        return motorKey;
    }

    @Override
    public MotorControllerKind controllerKind() {
        return controllerKind;
    }

    @Override
    public MotorTechnology technology() {
        return technology;
    }
}
