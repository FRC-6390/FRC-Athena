package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import java.util.Objects;

/**
 * What a control binding does when the last Action driving it ends and nothing replaces it.
 *
 * <ul>
 *   <li>{@link #NEUTRAL} (default): stop commanding; the motor keeps its declared neutral mode.</li>
 *   <li>{@link #HOLD}: keep closed-loop control of the last reference position (position) or hold
 *       zero velocity (velocity). Held without a lease until another Action commands the binding.</li>
 *   <li>{@link #DECELERATE}: brake at the profile's maximum deceleration to a stop, then hold. Position
 *       bindings without a profile, and velocity bindings, reach the stop by the velocity ramp.</li>
 *   <li>{@link #neutral(MotorNeutralMode)}: like NEUTRAL, with the neutral mode overridden until the
 *       next Action commands that motor.</li>
 * </ul>
 *
 * <p>Percent and voltage controls have no closed-loop target, so HOLD and DECELERATE behave as NEUTRAL
 * for them.</p>
 */
public final class Release {
    /** Brake to a stop at the profile's deceleration, then hold. */
    public static final Release DECELERATE = new Release(Kind.DECELERATE, null);
    /** Keep holding the last reference target. */
    public static final Release HOLD = new Release(Kind.HOLD, null);
    /** Stop commanding and keep the declared neutral mode. */
    public static final Release NEUTRAL = new Release(Kind.NEUTRAL, null);

    private final Kind kind;
    private final MotorNeutralMode neutralMode;

    private Release(Kind kind, MotorNeutralMode neutralMode) {
        this.kind = kind;
        this.neutralMode = neutralMode;
    }

    /** Neutral release that overrides the motor's neutral mode until the next Action commands it. */
    public static Release neutral(MotorNeutralMode mode) {
        return new Release(Kind.NEUTRAL, Objects.requireNonNull(mode, "mode"));
    }

    /** Returns the neutral mode override, or null when the declared mode applies. */
    public MotorNeutralMode neutralMode() {
        return neutralMode;
    }

    /** Returns true for policies that keep the binding under closed-loop control after release. */
    public boolean holds() {
        return kind == Kind.HOLD || kind == Kind.DECELERATE;
    }

    /** Returns true when the release brakes along the profile before holding. */
    public boolean decelerates() {
        return kind == Kind.DECELERATE;
    }

    @Override
    public String toString() {
        String base = switch (kind) {
            case DECELERATE -> "Release.DECELERATE";
            case HOLD -> "Release.HOLD";
            case NEUTRAL -> "Release.NEUTRAL";
        };
        return neutralMode == null ? base : base + "(" + neutralMode + ")";
    }

    private enum Kind { DECELERATE, HOLD, NEUTRAL }
}
