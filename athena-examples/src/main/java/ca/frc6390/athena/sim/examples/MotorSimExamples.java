package ca.frc6390.athena.sim.examples;

import ca.frc6390.athena.sim.Motor;
import ca.frc6390.athena.sim.MotorSimType;
import edu.wpi.first.math.system.plant.DCMotor;

/**
 * Example helpers for constructing DCMotor models in simulation code.
 */
public final class MotorSimExamples {
    private MotorSimExamples() {}

    public static DCMotor fromCatalog(Motor motor, int count) {
        return motor.createSimMotor(Math.max(1, count));
    }

    public static DCMotor fromFactory(MotorSimType type, int count) {
        return type.createSimMotor(Math.max(1, count));
    }
}
