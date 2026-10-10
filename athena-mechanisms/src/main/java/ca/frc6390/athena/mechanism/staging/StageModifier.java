package ca.frc6390.athena.mechanism.staging;

import ca.frc6390.athena.mechanism.core.ControlBinding;
import java.util.Objects;

/**
 * A cap applied to one control binding while a {@link Staging} stage is active.
 *
 * <ul>
 *   <li>{@link Kind#EFFORT} scales voltage and percent outputs after the control loop.</li>
 *   <li>{@link Kind#SPEED} scales the motion-profile maximum velocity, and scales velocity targets.</li>
 *   <li>{@link Kind#ACCELERATION} scales the motion-profile maximum acceleration.</li>
 * </ul>
 *
 * @param binding control binding the cap applies to
 * @param kind what is capped
 * @param fraction multiplier in {@code [0, 1]}
 */
public record StageModifier(ControlBinding<?> binding, Kind kind, double fraction) {
    /** Kind of cap. */
    public enum Kind { EFFORT, SPEED, ACCELERATION }

    public StageModifier {
        Objects.requireNonNull(binding, "binding");
        Objects.requireNonNull(kind, "kind");
        if (!(fraction >= 0.0 && fraction <= 1.0)) {
            throw new IllegalArgumentException("Stage limit fraction must be in [0, 1].");
        }
    }
}
