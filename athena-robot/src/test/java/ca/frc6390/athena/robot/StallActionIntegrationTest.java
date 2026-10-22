package ca.frc6390.athena.robot;

import edu.wpi.first.units.Units;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.signal.MotorStallSignal;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import org.junit.jupiter.api.Test;

class StallActionIntegrationTest {
    private static final double DT = 0.02;

    @Test
    void homingDrivesUntilStallThenZeroesAndIdles() {
        SimulationSession simulation = SimulationSession.create();
        HomingArm arm = new HomingArm();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(arm);
        runtime.request(arm.home);

        double now = 0.0;
        for (int tick = 0; tick < 5; tick++, now += DT) runtime.teleopPeriodic(now, DT);
        assertDrivingIn(simulation, arm.motor);

        simulation.motor(arm.motor).locked(true); // reached the hard stop
        for (int tick = 0; tick < 15; tick++, now += DT) runtime.teleopPeriodic(now, DT);

        assertEquals(0.0, simulation.motor(arm.motor).commandValue(), 1.0e-9, "should idle after stall");
        assertEquals(1, arm.zeroed[0], "homing should zero exactly once");
    }

    @Test
    void stallDebounceRestartsEachRun() {
        SimulationSession simulation = SimulationSession.create();
        HomingArm arm = new HomingArm();
        RobotRuntime runtime = RobotRuntime.simulated(simulation).register(arm);
        simulation.motor(arm.motor).locked(true);
        double now = 0.0;

        runtime.request(arm.bringIn);
        for (int tick = 0; tick < 15; tick++, now += DT) runtime.teleopPeriodic(now, DT);
        assertEquals(0.0, simulation.motor(arm.motor).commandValue(), 1.0e-9);

        // Second run: must drive again for the debounce window instead of finishing instantly.
        runtime.request(arm.bringIn);
        runtime.teleopPeriodic(now, DT);
        now += DT;
        assertDrivingIn(simulation, arm.motor);
    }

    private static void assertDrivingIn(SimulationSession simulation, MotorDevice motor) {
        var handle = simulation.motor(motor);
        double volts = handle.appliedVoltage();
        assertTrue(volts < 0.0 && volts >= -0.3 * simulation.busVoltageVolts(), "should be driving in");
        var constants = MotorConstants.forKind(motor.kind()).orElseThrow();
        double backEmf = handle.integratedVelocityRotationsPerSecond() * 2.0 * Math.PI
                / constants.raw().speedConstantRadPerSecPerVolt();
        double windingAmps = Math.abs((volts - backEmf) / constants.resistance().in(Units.Ohms));
        assertTrue(windingAmps <= 40.0 + 1e-8, "the homing action respects its winding-current limit");
        assertTrue(Math.abs(volts) * windingAmps / simulation.busVoltageVolts() <= 30.0 + 1e-8);
    }

    private static final class HomingArm implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 81)
                .supplyCurrentLimit(Units.Amps.of(30))
                .statorCurrentLimit(Units.Amps.of(40));
        private final MotorStallSignal stall = motor.stall().forTime(Units.Seconds.of(0.08));
        private final int[] zeroed = {0};
        private final Action bringIn = Actions.untilStalled(stall, motor.percent(-0.3)).then(motor.neutral());
        private final Action home = Actions.sequence()
                .untilStalled(stall, motor.percent(-0.3))
                .doOnce(() -> zeroed[0]++)
                .then(motor.neutral());
    }
}
