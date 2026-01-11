package ca.frc6390.athena.sim;

import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Core simulation motor factory used across vendor modules.
 */
@FunctionalInterface
public interface MotorSimType {
    DCMotor createSimMotor(int count);
}
