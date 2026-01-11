package ca.frc6390.athena.mechanisms.sim;

/**
 * Represents a lightweight physics adapter for a {@link ca.frc6390.athena.mechanisms.Mechanism}.
 * Implementations receive the commanded output, advance their internal model, then push the
 * resulting sensor states (encoders, limit switches, etc.) back into the mechanism.
 */
public interface MechanismSimulationModel {

    /**
     * Advance the simulation by {@code dtSeconds} seconds.
     *
     * @param dtSeconds elapsed time since the previous update, in seconds
     */
    void update(double dtSeconds);

    /**
     * Reset any internal state and push the mechanism's initial sensor readings. Called once when the
     * model is registered and whenever {@link #reset()} is triggered manually.
     */
    default void reset() {
        // no-op by default
    }
}
