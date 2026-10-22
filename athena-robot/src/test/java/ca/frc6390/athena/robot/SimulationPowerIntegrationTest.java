package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.sim.runtime.SimulationSession;
import edu.wpi.first.units.Units;
import org.junit.jupiter.api.Test;

class SimulationPowerIntegrationTest {
    @Test
    void standaloneSimulationSuppliesItsNominalBusAndEnforcesSupplyInTheCorrectDomain() {
        SimulationSession simulation = SimulationSession.create();
        PoweredAxis axis = new PoweredAxis();
        try (RobotRuntime runtime = RobotRuntime.simulated(simulation).register(axis)) {
            runtime.request(axis.initial);
            runtime.teleopPeriodic(0.0, 0.02);
            assertEquals(simulation.busVoltageVolts(), runtime.powerSnapshot().busVolts());
            assertEquals(12.0, runtime.powerSnapshot().busVolts());
            assertFalse(runtime.powerSnapshot().shedding());
            double resistance = MotorConstants.forKind(axis.motor.kind()).orElseThrow().resistance().in(Units.Ohms);
            double cap = runtime.powerSnapshot().motors().get(0).capAmps();
            assertEquals(Math.sqrt(12.0 * 5.0 / resistance), cap, 1e-9);
            assertTrue(cap > 5.0, "partial-duty supply allowance exceeds equal numeric phase amps");
            assertEquals(Math.sqrt(12.0 * 5.0 * resistance), simulation.motor(axis.motor).appliedVoltage(), 1e-9);
        }
    }

    @Test
    void explicitPowerBusSupplierOverridesTheSimulationDefault() {
        SimulationSession simulation = SimulationSession.create();
        PoweredAxis axis = new PoweredAxis();
        try (RobotRuntime runtime = RobotRuntime.simulated(simulation).powerBusVoltage(() -> 6.0).register(axis)) {
            runtime.request(axis.initial);
            runtime.teleopPeriodic(0.0, 0.02);
            assertEquals(6.0, runtime.powerSnapshot().busVolts());
            assertTrue(runtime.powerSnapshot().shedding(), "a measured 6 V bus is already below the 7 V floor");
            assertEquals(0.0, simulation.motor(axis.motor).appliedVoltage(), 1e-9,
                    "candidate allocation must not add positive load below the battery floor");
        }
    }

    @Test
    void genuinelyMissingBusTelemetryKeepsTheConservativeZeroSupplyAllowance() {
        SimulationSession simulation = SimulationSession.create();
        PoweredAxis axis = new PoweredAxis();
        try (RobotRuntime runtime = RobotRuntime.simulated(simulation).powerBusVoltage(() -> Double.NaN).register(axis)) {
            runtime.request(axis.initial);
            runtime.teleopPeriodic(0.0, 0.02);
            assertTrue(Double.isNaN(runtime.powerSnapshot().busVolts()));
            assertEquals(0.0, runtime.powerSnapshot().motors().get(0).capAmps());
            assertEquals(0.0, simulation.motor(axis.motor).appliedVoltage());
            assertEquals("Supply limit unavailable", runtime.powerSnapshot().motors().get(0).status());
        }
    }

    private static final class PoweredAxis implements Mechanism {
        private final MotorDevice motor = MotorDevice.of(MotorKinds.NEO, 73)
                .currentLimit(Units.Amps.of(0)).supplyCurrentLimit(Units.Amps.of(5));
        private final Action initial = motor.voltage(Units.Volts.of(6.0));
    }
}
