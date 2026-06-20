package ca.frc6390.athena.sim.mechanism;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.mechanism.config.Mechanisms;
import ca.frc6390.athena.sim.world.SimWorld;

class SimMechanismTest {
    @Test
    void percentOutputStateDrivesSimulatedMotorVelocity() {
        SimWorld world = new SimWorld();
        var mechanism = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .control(control -> control.percentOutput())
                .state("feed", state -> state.target(0.5))
                .toSpec();

        var sim = new SimMechanism(world, mechanism)
                .percentOutputVelocityScale(3.0)
                .applyState("feed");
        world.step(2.0);

        assertEquals(0.5, sim.motor("roller").percentOutput(), 1.0e-9);
        assertEquals(1.5, sim.motor("roller").velocityPerSecond(), 1.0e-9);
        assertEquals(3.0, sim.motor("roller").position(), 1.0e-9);
    }

    @Test
    void velocityStateSetsSimulatedMotorVelocity() {
        SimWorld world = new SimWorld();
        var mechanism = Mechanisms.flywheel("shooter")
                .motor("leader", motor -> motor.hardware(AthenaMotor.SIM, 2))
                .control(control -> control.velocity(pid -> pid.p(0.1)))
                .state("speaker", state -> state.target(4600.0))
                .toSpec();

        var sim = new SimMechanism(world, mechanism).applyState("speaker");
        world.step(0.25);

        assertEquals(4600.0, sim.motor("leader").velocityPerSecond(), 1.0e-9);
        assertEquals(1150.0, sim.motor("leader").position(), 1.0e-9);
    }

    @Test
    void positionStateSetsSimulatedMotorPosition() {
        SimWorld world = new SimWorld();
        var mechanism = Mechanisms.simple("arm")
                .motor("pivot", motor -> motor.hardware(AthenaMotor.SIM, 3))
                .encoder("absolute", encoder -> encoder.hardware(AthenaEncoder.SIM, 3))
                .positionSource("absolute")
                .control(control -> control.position(pid -> pid.p(0.2)))
                .state("amp", state -> state.target(61.0))
                .toSpec();

        var sim = new SimMechanism(world, mechanism).applyState("amp");

        assertEquals(61.0, sim.motor("pivot").position(), 1.0e-9);
        assertEquals(0.0, sim.motor("pivot").velocityPerSecond(), 1.0e-9);
    }

    @Test
    void rejectsUnknownStateAndMotorNames() {
        SimWorld world = new SimWorld();
        var mechanism = Mechanisms.simple("intake")
                .motor("roller", motor -> motor.hardware(AthenaMotor.SIM, 1))
                .state("feed", state -> state.target(0.5))
                .toSpec();
        var sim = new SimMechanism(world, mechanism);

        assertThrows(IllegalArgumentException.class, () -> sim.applyState("missing"));
        assertThrows(IllegalArgumentException.class, () -> sim.motor("missing"));
    }
}
