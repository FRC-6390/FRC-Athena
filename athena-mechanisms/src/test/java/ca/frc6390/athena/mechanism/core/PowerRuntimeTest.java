package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.power.PowerPolicy;
import ca.frc6390.athena.hardware.power.PowerProfile;
import ca.frc6390.athena.hardware.power.PowerSnapshot;
import ca.frc6390.athena.hardware.power.Thermal;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.RuntimeScope;
import edu.wpi.first.units.Units;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;

class PowerRuntimeTest {
    private static final double DT = 0.02;

    /** Motor with settable electrical state that records every runtime configuration write (there must be none). */
    private static final class PowerMotor implements MotorHandle {
        final MotorDevice device;
        double appliedVolts;
        double supplyAmps;
        double statorAmps;
        final List<MotorRuntimeConfig> writes = new ArrayList<>();

        PowerMotor(MotorDevice device) { this.device = device; }

        @Override public MotorDevice device() { return device; }
        @Override public boolean supportsRuntimeConfiguration() { return true; }
        @Override public void applyRuntimeConfiguration(MotorRuntimeConfig configuration) { writes.add(configuration); }
        @Override public double appliedVoltage() { return appliedVolts; }
        @Override public double supplyCurrentAmps() { return supplyAmps; }
        @Override public double statorCurrentAmps() { return statorAmps; }
        @Override public double integratedVelocityRotationsPerSecond() { return 0.0; }
        @Override public OptionalDouble temperatureCelsius() { return OptionalDouble.empty(); }
    }

    private static final class Hardware implements ActionContext {
        final Map<MotorDevice, PowerMotor> motors = new HashMap<>();
        @Override public PowerMotor motor(MotorDevice motor) {
            return motors.computeIfAbsent(motor, PowerMotor::new);
        }
    }

    /** Runs the pass with a fixed bus voltage and no battery model, so the planner's inputs are explicit. */
    private static PowerRuntime runtime(double busVolts, PowerPolicy policy) {
        PowerRuntime power = new PowerRuntime();
        power.policy(policy);
        power.busVoltage(() -> busVolts);
        return power;
    }

    @Test
    void sagShedsTheCapWithoutAnyConfigurationWrite() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 71).currentLimit(Units.Amps.of(400));
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        handle.appliedVolts = 12.0;               // demand = 12 / 0.033 ~= 364 A
        handle.supplyAmps = 364.0;
        RuntimeOverrides overrides = new RuntimeOverrides();
        PowerRuntime power = new PowerRuntime();
        power.policy(PowerPolicy.defaults().estimateBatteryResistance(false));
        List<MotorDevice> motors = List.of(motor);

        power.busVoltage(() -> 6.0);
        power.cycle(motors, hardware, overrides, 0.0);
        double shed = power.capAmps(motor);
        assertTrue(shed > 0 && shed < 400, "cap shed below declared 400 A: " + shed);
        assertEquals("Power shed", power.snapshot().motors().get(0).status());

        power.cycle(motors, hardware, overrides, DT);
        assertTrue(handle.writes.isEmpty(), "no configuration write while shedding");

        // Bus recovers: the cap returns to the declared limit.
        power.busVoltage(() -> 12.5);
        handle.supplyAmps = 30.0;
        handle.appliedVolts = 1.0;
        power.cycle(motors, hardware, overrides, 0.3);
        assertEquals(Double.POSITIVE_INFINITY, power.capAmps(motor), 0.0, "back at the declared limit: no clamp");
        assertTrue(handle.writes.isEmpty(), "no configuration write on recovery");
    }

    @Test
    void userConfigurationChangeReachesTheCapOnTheNextCycle() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 72).currentLimit(Units.Amps.of(60));
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        RuntimeOverrides overrides = new RuntimeOverrides();
        PowerRuntime power = runtime(12.5, PowerPolicy.defaults());
        List<MotorDevice> motors = List.of(motor);
        power.cycle(motors, hardware, overrides, 0.0);
        assertEquals(Double.POSITIVE_INFINITY, power.capAmps(motor), 0.0, "cap at the declared limit is not clamped");

        overrides.supplyLimit(motor, 80);
        power.cycle(motors, hardware, overrides, DT);
        assertEquals(Double.POSITIVE_INFINITY, power.capAmps(motor), 0.0, "the user's 80 A is the new declared limit");
        assertTrue(handle.writes.isEmpty());

        assertTrue(handle.writes.isEmpty());
    }

    @Test
    void handlesWithoutRuntimeConfigurationAreCappedAndNeverWritten() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 73).currentLimit(Units.Amps.of(400));
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        handle.appliedVolts = 12.0;
        handle.supplyAmps = 364.0;
        PowerRuntime power = runtime(6.0, PowerPolicy.defaults().estimateBatteryResistance(false));
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), 0.0);
        assertTrue(handle.writes.isEmpty());
        assertTrue(power.capAmps(motor) < 400.0, "the cap still applies through the output path");
        assertEquals("Power shed", power.snapshot().motors().get(0).status());
    }

    @Test
    void profileSustainedCapIsPlannedAndReportedWithoutAWrite() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 74).currentLimit(Units.Amps.of(400))
                .power(PowerProfile.of().sustained(Units.Amps.of(60)));
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        handle.appliedVolts = 12.0;
        handle.supplyAmps = 364.0;
        PowerRuntime power = runtime(12.5, PowerPolicy.defaults().estimateBatteryResistance(false));
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), 0.0);
        assertEquals(60.0, power.capAmps(motor), 1e-9);
        PowerSnapshot snapshot = power.snapshot();
        assertEquals(1, snapshot.motors().size());
        assertEquals(60.0, snapshot.motors().get(0).capAmps(), 1e-9);
        assertEquals("Power capped", snapshot.motors().get(0).status());
        assertTrue(handle.writes.isEmpty());
    }

    @Test
    void thermalDerateScalesTheCapWhileHot() throws Exception {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 75).currentLimit(Units.Amps.of(100))
                .thermal(Thermal.derate(Units.Celsius.of(40), Units.Celsius.of(60), 0.5));
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        handle.statorAmps = 200.0;
        handle.supplyAmps = 0.0;
        handle.appliedVolts = 0.0;
        RuntimeOverrides overrides = new RuntimeOverrides();
        // Bind the declaration to a runtime so its lumped thermal model exists and is heated.
        AutoCloseable binding = motor.bindRuntime(new RuntimeScope("power-derate-test"), handle);
        try {
            PowerRuntime power = runtime(12.5, PowerPolicy.defaults());
            List<MotorDevice> motors = List.of(motor);
            double now = 0.0;
            for (int cycle = 0; cycle < 5000 && motor.temperature().in(Units.Celsius) < 60.0; cycle++) {
                power.cycle(motors, hardware, overrides, now);
                now += DT;
            }
            assertTrue(motor.temperature().in(Units.Celsius) >= 60.0, "motor heated: " + motor.temperature().in(Units.Celsius));
            power.cycle(motors, hardware, overrides, now);
            double expected = 100 * motor.thermalCurve().derateFraction(motor.temperature());
            assertEquals(expected, power.capAmps(motor), 1e-6);
            assertTrue(power.capAmps(motor) < 100.0, "derated below declared 100 A");
            assertEquals("Thermal derate", power.snapshot().motors().get(0).status());
            assertTrue(handle.writes.isEmpty());
        } finally {
            binding.close();
        }
    }
}
