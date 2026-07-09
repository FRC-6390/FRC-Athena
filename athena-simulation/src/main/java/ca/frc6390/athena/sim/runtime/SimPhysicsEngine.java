package ca.frc6390.athena.sim.runtime;

import ca.frc6390.athena.hardware.sim.SimModel;
import java.util.Collection;

/**
 * Physics engine used by a simulation session to advance registered models.
 */
@FunctionalInterface
public interface SimPhysicsEngine {
    /**
     * Advances registered simulation models.
     *
     * @param models registered models
     * @param session owning simulation session
     * @param seconds timestep in seconds
     */
    void step(Collection<SimModel> models, SimulationSession session, double seconds);
}
