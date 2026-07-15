package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.control.FeedforwardGains;
import ca.frc6390.athena.mechanism.control.PidGains;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import ca.frc6390.athena.mechanism.core.Controls;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.TelemetryValue;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import org.junit.jupiter.api.Test;

class MechanismTelemetryTest {
    @Test
    void discoversExistingDeclarationsAndAppliesRuntimeDisable() {
        TunedMechanism mechanism = new TunedMechanism();
        SimulationSession simulation = SimulationSession.create();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(mechanism);

        assertTrue(runtime.mechanismTelemetry().containsKey("tunedMechanism/motor/disabled"));
        assertTrue(runtime.mechanismTelemetry().containsKey("tunedMechanism/control/disabled"));
        assertTrue(runtime.mechanismTelemetry().containsKey("tunedMechanism/control/pid/p"));
        assertTrue(runtime.mechanismTelemetry().containsKey("tunedMechanism/control/feedforward/velocity"));
        assertTrue(runtime.mechanismTelemetry().containsKey("tunedMechanism/position"));

        runtime.mechanismTelemetry().get("tunedMechanism/control/pid/p").set(3.0);
        runtime.mechanismTelemetry().get("tunedMechanism/control/feedforward/velocity").set(1.5);
        assertEquals(3.0, mechanism.pid.p(), 1e-9);
        assertEquals(1.5, mechanism.feedforward.velocityGain(), 1e-9);

        runtime.request(mechanism.run);
        runtime.robotPeriodic(0.0, 0.02);
        assertEquals(6.0, simulation.motor(mechanism.motor).appliedVoltage(), 1e-9);

        runtime.mechanismTelemetry().get("tunedMechanism/motor/disabled").set(true);
        runtime.robotPeriodic(0.02, 0.02);
        assertEquals(0.0, simulation.motor(mechanism.motor).appliedVoltage(), 1e-9);
    }

    private static final class TunedMechanism implements Mechanism {
        final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        final PidGains pid = PidGains.of(1.0, 0.0, 0.0);
        final FeedforwardGains feedforward = FeedforwardGains.of(0.0, 1.0, 0.0, 0.0);
        final ControlBinding control = Controls.position(motor)
                .feedback(motor.encoder()).pid(pid).feedforward(feedforward);
        final TelemetryValue position = TelemetryValue.number(motor.encoder()::position);
        final Action run = motor.percent(0.5);
    }
}
