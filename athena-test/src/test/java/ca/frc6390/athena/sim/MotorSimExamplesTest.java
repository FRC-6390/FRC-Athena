package ca.frc6390.athena.sim;

import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.sim.examples.MotorSimExamples;
import edu.wpi.first.math.system.plant.DCMotor;
import org.junit.jupiter.api.Test;

final class MotorSimExamplesTest {

    @Test
    void catalogFactoryBuildsMotorModel() {
        DCMotor model = MotorSimExamples.fromCatalog(Motor.CIM, 2);
        assertNotNull(model);
        assertTrue(Double.isFinite(model.getCurrent(0.0, 12.0)));
    }

    @Test
    void customFactoryBuildsMotorModel() {
        DCMotor model = MotorSimExamples.fromFactory(count -> DCMotor.getNEO(count), 1);
        assertNotNull(model);
        assertTrue(Double.isFinite(model.getCurrent(0.0, 12.0)));
    }
}
