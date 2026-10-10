package ca.frc6390.athena.mechanism.staging;

/**
 * Combined caps for one control binding in the current cycle. Each factor is in {@code [0, 1]}; 1 means no cap.
 *
 * @param effort multiplier on voltage and percent output
 * @param speed multiplier on maximum velocity
 * @param acceleration multiplier on maximum acceleration
 */
public record StageCaps(double effort, double speed, double acceleration) {
    /** No caps. */
    public static final StageCaps NONE = new StageCaps(1.0, 1.0, 1.0);

    /** Returns these caps with a further cap applied; the smaller factor wins for each kind. */
    public StageCaps with(StageModifier.Kind kind, double fraction) {
        return switch (kind) {
            case EFFORT -> new StageCaps(Math.min(effort, fraction), speed, acceleration);
            case SPEED -> new StageCaps(effort, Math.min(speed, fraction), acceleration);
            case ACCELERATION -> new StageCaps(effort, speed, Math.min(acceleration, fraction));
        };
    }
}
