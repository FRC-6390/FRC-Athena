package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
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
        assertTrue(runtime.mechanismTelemetry().containsKey("tunedMechanism/motor/positionRotations"));
        assertTrue(runtime.mechanismTelemetry().containsKey("tunedMechanism/motor/appliedVoltage"));
        assertTrue(runtime.mechanismTelemetry().containsKey("tunedMechanism/encoder/absolutePosition"));
        assertTrue(runtime.mechanismTelemetry().containsKey("tunedMechanism/control/feedback/position"));
        assertTrue(runtime.mechanismTelemetry().containsKey("tunedMechanism/control/motors/0/command/value"));

        runtime.mechanismTelemetry().get("tunedMechanism/control/pid/p").set(3.0);
        runtime.mechanismTelemetry().get("tunedMechanism/control/feedforward/velocity").set(1.5);
        assertEquals(3.0, mechanism.pid.p(), 1e-9);
        assertEquals(1.5, mechanism.feedforward.velocityGain(), 1e-9);

        runtime.request(mechanism.run);
        runtime.robotPeriodic(0.0, 0.02);
        assertEquals(6.0, simulation.motor(mechanism.motor).appliedVoltage(), 1e-9);
        simulation.motor(mechanism.motor).state(2.5, 4.0);
        assertEquals(2.5, runtime.mechanismTelemetry()
                .get("tunedMechanism/motor/positionRotations").number(), 1e-9);
        assertEquals(4.0, runtime.mechanismTelemetry()
                .get("tunedMechanism/control/feedback/velocity").number(), 1e-9);
        assertEquals(0.5, runtime.mechanismTelemetry()
                .get("tunedMechanism/control/motors/0/command/value").number(), 1e-9);

        runtime.mechanismTelemetry().get("tunedMechanism/motor/disabled").set(true);
        runtime.robotPeriodic(0.02, 0.02);
        assertEquals(0.0, simulation.motor(mechanism.motor).appliedVoltage(), 1e-9);
    }

    @Test
    void tracesIdleMotorsDeclaredOnlyThroughAControlBinding() {
        ControlOnlyMechanism mechanism = new ControlOnlyMechanism();
        RobotRuntime runtime = RobotRuntime.simulated(SimulationSession.create()).register(mechanism);

        runtime.robotPeriodic(0.0, 0.02);

        assertEquals(1, runtime.mechanismTraces().get(0).motors().size());
        assertEquals(mechanism.control.output().defaultName(),
                runtime.mechanismTraces().get(0).motors().get(0).name());
        assertTrue(runtime.mechanismTelemetry()
                .containsKey("controlOnlyMechanism/control/motors/0/positionRotations"));
    }

    private static final class TunedMechanism implements Mechanism {
        final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        final EncoderDevice encoder = motor.encoder();
        final PidGains pid = PidGains.of(1.0, 0.0, 0.0);
        final FeedforwardGains feedforward = FeedforwardGains.of(0.0, 1.0, 0.0, 0.0);
        final ControlBinding control = Controls.position(motor)
                .feedback(encoder).pid(pid).feedforward(feedforward);
        final TelemetryValue position = TelemetryValue.number(encoder::position);
        final Action run = motor.percent(0.5);
    }

    private static final class ControlOnlyMechanism implements Mechanism {
        final ControlBinding control = Controls.velocity(MotorDevice.of(MotorKinds.KRAKEN_X60, 9));
    }
}
