package ca.frc6390.athena.api.hardware;

/** Common built-in motor-controller families known to Athena. */
public enum MotorControllerKinds implements MotorControllerKind {
    TALON_FX("ctre:talon-fx"),
    TALON_FXS("ctre:talon-fxs"),
    SPARK_MAX("rev:spark-max"),
    SPARK_FLEX("rev:spark-flex");

    private final String key;

    MotorControllerKinds(String key) {
        this.key = key;
    }

    @Override
    public String key() {
        return key;
    }

    @Override
    public boolean supports(MotorKind motor) {
        if (!(motor.motorKind() instanceof MotorKinds model)) {
            return true;
        }
        return switch (this) {
            case TALON_FX -> model == MotorKinds.FALCON_500
                    || model == MotorKinds.KRAKEN_X60
                    || model == MotorKinds.KRAKEN_X44;
            case TALON_FXS -> model != MotorKinds.FALCON_500
                    && model != MotorKinds.KRAKEN_X60
                    && model != MotorKinds.KRAKEN_X44;
            case SPARK_MAX, SPARK_FLEX -> model != MotorKinds.FALCON_500
                    && model != MotorKinds.KRAKEN_X60
                    && model != MotorKinds.KRAKEN_X44
                    && model != MotorKinds.MINION;
        };
    }
}
