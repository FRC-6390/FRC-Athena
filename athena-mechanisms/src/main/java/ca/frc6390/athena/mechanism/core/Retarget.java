package ca.frc6390.athena.mechanism.core;

/**
 * How a new target is planned when it arrives while a motion profile is already running
 * (a dynamic target change, or a different Action taking over the same control binding).
 *
 * <p>Binding default via {@link ControlBinding#retarget(Retarget)}; per Action via
 * {@code action.retarget(...)}. Applies to position and velocity controls with a motion profile
 * or velocity ramp.</p>
 */
public enum Retarget {
    /**
     * Re-plan from the current <em>reference</em> position and velocity. The profile never steps
     * and never restarts from rest. A takeover inherits the reference state.
     */
    SMOOTH,
    /**
     * Re-plan from the <em>measured</em> position and velocity, allowing the profile to brake at its
     * full deceleration. Used when the reference is known to be wrong (e.g. after a jam or a reseed).
     */
    IMMEDIATE
}
