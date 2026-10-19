package ca.frc6390.athena.sim.runtime;

import edu.wpi.first.units.Units;
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

        assertTrue(injector.motor(motor.defaultName(), Units.Rotations.of(2.5), Units.RotationsPerSecond.of(1.25),
                Units.Volts.of(11.0), Units.Amps.of(30.0), Units.Amps.of(45.0)));

        assertEquals(2.5, handle.integratedPositionRotations(), 0.0);
        assertEquals(1.25, handle.integratedVelocityRotationsPerSecond(), 0.0);
        assertEquals(11.0, handle.appliedVoltage(), 0.0);
        assertEquals(30.0, handle.supplyCurrentAmps(), 0.0);
        assertEquals(45.0, handle.statorCurrentAmps(), 0.0);
        assertFalse(injector.motor("no-such-motor", Units.Rotations.of(0), Units.RotationsPerSecond.of(0), Units.Volts.of(0),
                Units.Amps.of(0), Units.Amps.of(0)));
    }

    @Test
    void injectedReadingsEndAtTheNextPhysicsStep() {
        SimulationSession session = SimulationSession.create();
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X44, 10);
        SimMotorHandle handle = session.motor(motor);
        session.injector().motor(motor.defaultName(), Units.Rotations.of(0.0), Units.RotationsPerSecond.of(0.0),
                Units.Volts.of(9.0), Units.Amps.of(20.0), Units.Amps.of(25.0));
        assertEquals(9.0, handle.appliedVoltage(), 0.0);

        handle.clearInjected();
        assertEquals(0.0, handle.appliedVoltage(), 0.0);
    }
}
