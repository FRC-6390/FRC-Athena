package ca.frc6390.athena.rev;

import ca.frc6390.athena.sim.MotorSimType;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * REV-specific motor models for simulation.
 */
public enum RevMotor implements MotorSimType {
    NEO_V1(count -> DCMotor.getNEO(count)),
    NEO_VORTEX(count -> DCMotor.getNeoVortex(count));

    private final MotorSimType factory;

    RevMotor(MotorSimType factory) {
        this.factory = factory;
    }

    @Override
    public DCMotor createSimMotor(int count) {
        return factory.createSimMotor(count);
    }
}
