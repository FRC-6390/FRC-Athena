package ca.frc6390.athena.sim.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.sim.hardware.SimMotorHandle;
import org.junit.jupiter.api.Test;

class SimulationInjectorTest {
    @Test
    void injectsMotorFeedbackAndElectricalReadingsByName() {
        SimulationSession session = SimulationSession.create();
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X44, 9);
        SimMotorHandle handle = session.motor(motor);
        SimulationInjector injector = session.injector();

        assertTrue(injector.motor(motor.defaultName(), 2.5, 1.25, 11.0, 30.0, 45.0));

        assertEquals(2.5, handle.integratedPositionRotations(), 0.0);
        assertEquals(1.25, handle.integratedVelocityRotationsPerSecond(), 0.0);
        assertEquals(11.0, handle.appliedVoltage(), 0.0);
        assertEquals(30.0, handle.supplyCurrentAmps(), 0.0);
        assertEquals(45.0, handle.statorCurrentAmps(), 0.0);
        assertFalse(injector.motor("no-such-motor", 0, 0, 0, 0, 0));
    }

    @Test
    void injectedReadingsEndAtTheNextPhysicsStep() {
        SimulationSession session = SimulationSession.create();
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X44, 10);
        SimMotorHandle handle = session.motor(motor);
        session.injector().motor(motor.defaultName(), 0.0, 0.0, 9.0, 20.0, 25.0);
        assertEquals(9.0, handle.appliedVoltage(), 0.0);

        handle.clearInjected();
        assertEquals(0.0, handle.appliedVoltage(), 0.0);
    }
}
