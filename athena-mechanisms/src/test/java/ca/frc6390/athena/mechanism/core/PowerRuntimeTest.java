package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
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

    /** Motor with settable electrical state that records every runtime configuration write. */
    private static final class PowerMotor implements MotorHandle {
        final MotorDevice device;
        double appliedVolts;
        double supplyAmps;
        double statorAmps;
        boolean supportsConfig = true;
        final List<MotorRuntimeConfig> writes = new ArrayList<>();

        PowerMotor(MotorDevice device) { this.device = device; }

        @Override public MotorDevice device() { return device; }
        @Override public boolean supportsRuntimeConfiguration() { return supportsConfig; }
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
    void sagShedsCurrentLimitThenRestoresAfterRecovery() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 71).currentLimit(400);
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
        assertEquals(1, handle.writes.size(), "shed write");
        int shedLimit = handle.writes.get(0).supplyCurrentLimitAmps();
        assertTrue(shedLimit > 0 && shedLimit < 400, "limit shed below declared 400 A: " + shedLimit);

        // Same condition 20 ms later: inside the minimum write interval, so no new configuration write.
        power.cycle(motors, hardware, overrides, DT);
        assertEquals(1, handle.writes.size(), "throttled");

        // Bus recovers and the interval has passed: the declared limit comes back.
        power.busVoltage(() -> 12.5);
        handle.supplyAmps = 30.0;
        handle.appliedVolts = 1.0;
        power.cycle(motors, hardware, overrides, 0.3);
        assertEquals(2, handle.writes.size());
        assertEquals(400, handle.writes.get(1).supplyCurrentLimitAmps());
        assertEquals(MotorNeutralMode.COAST, handle.writes.get(1).neutralMode());
    }

    @Test
    void userConfigurationChangeIsWrittenImmediately() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 72).currentLimit(60);
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        RuntimeOverrides overrides = new RuntimeOverrides();
        PowerRuntime power = runtime(12.5, PowerPolicy.defaults());
        List<MotorDevice> motors = List.of(motor);
        power.cycle(motors, hardware, overrides, 0.0);
        assertTrue(handle.writes.isEmpty(), "declared config already applied");

        overrides.supplyLimit(motor, 80);
        power.cycle(motors, hardware, overrides, DT);
        assertFalse(handle.writes.isEmpty());
        assertEquals(80, handle.writes.get(handle.writes.size() - 1).supplyCurrentLimitAmps());
    }

    @Test
    void unsupportedHandlesAreNeverWritten() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 73).currentLimit(400);
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        handle.supportsConfig = false;
        handle.appliedVolts = 12.0;
        handle.supplyAmps = 364.0;
        PowerRuntime power = runtime(6.0, PowerPolicy.defaults().estimateBatteryResistance(false));
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), 0.0);
        assertTrue(handle.writes.isEmpty());
        assertEquals("Unsupported", power.snapshot().motors().get(0).status());
    }

    @Test
    void profileSustainedCapIsWrittenAndSnapshotReportsIt() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 74).currentLimit(400)
                .power(PowerProfile.of().sustained(Units.Amps.of(60)));
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        handle.appliedVolts = 12.0;
        handle.supplyAmps = 364.0;
        PowerRuntime power = runtime(12.5, PowerPolicy.defaults().estimateBatteryResistance(false));
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), 0.0);
        assertEquals(60, handle.writes.get(0).supplyCurrentLimitAmps());
        PowerSnapshot snapshot = power.snapshot();
        assertEquals(1, snapshot.motors().size());
        assertEquals(60.0, snapshot.motors().get(0).capAmps(), 1e-9);
        assertEquals("Power capped", snapshot.motors().get(0).status());
    }

    @Test
    void thermalDerateScalesDeclaredLimitWhileHot() throws Exception {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 75).currentLimit(100)
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
            for (int cycle = 0; cycle < 5000 && motor.temperatureCelsius() < 60.0; cycle++) {
                power.cycle(motors, hardware, overrides, now);
                now += DT;
            }
            assertTrue(motor.temperatureCelsius() >= 60.0, "motor heated: " + motor.temperatureCelsius());
            // Let the writes settle: at most one write per 0.25 s, and only for changes of 2 A or more.
            for (int cycle = 0; cycle < 30; cycle++) {
                power.cycle(motors, hardware, overrides, now);
                now += DT;
            }
            assertFalse(handle.writes.isEmpty(), "derated limit written");
            int expected = (int) Math.round(100 * motor.thermalCurve().derateFraction(motor.temperatureCelsius()));
            int written = handle.writes.get(handle.writes.size() - 1).supplyCurrentLimitAmps();
            assertTrue(Math.abs(written - expected) <= 2, "written " + written + " expected " + expected);
            assertTrue(written < 100, "derated below declared 100 A");
            assertEquals("Thermal derate", power.snapshot().motors().get(0).status());
        } finally {
            binding.close();
        }
    }
}
