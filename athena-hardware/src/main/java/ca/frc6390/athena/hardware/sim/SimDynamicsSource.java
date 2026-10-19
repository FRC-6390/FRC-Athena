package ca.frc6390.athena.hardware.sim;

import edu.wpi.first.units.measure.MomentOfInertia;
import java.util.Optional;

/**
 * Plant parameters a simulation model can take from an identified or declared source.
 *
 * <p>{@link SimModel#model(SimDynamicsSource)} applies the values it provides. An empty value leaves
 * the model's own default unchanged.</p>
 */
public interface SimDynamicsSource {
    /** Returns the mechanism-side moment of inertia about its motion axis, when the source provides one. */
    Optional<MomentOfInertia> momentOfInertia();

    /**
     * Returns the identified plant: friction, gravity, viscous and inertia terms the simulation applies
     * instead of the motor's own dynamics. Empty when the source does not identify one.
     */
    default Optional<SimPlant> plant() {
        return Optional.empty();
    }
}
