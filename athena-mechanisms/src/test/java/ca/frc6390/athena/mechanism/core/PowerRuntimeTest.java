package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorControllerKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.power.PowerPolicy;
import ca.frc6390.athena.hardware.power.PowerProfile;
import ca.frc6390.athena.hardware.power.PowerSnapshot;
import ca.frc6390.athena.hardware.power.Thermal;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.hardware.runtime.MotorGroupPolicy;
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
        double supplyAmps = Double.NaN;
        double statorAmps = Double.NaN;
        double rotorRps;
        boolean faulted;
        final List<MotorRuntimeConfig> writes = new ArrayList<>();

        PowerMotor(MotorDevice device) { this.device = device; }

        @Override public MotorDevice device() { return device; }
        @Override public boolean supportsRuntimeConfiguration() { return true; }
        // This fixture emulates the real native capability, rather than assuming every custom handle
        // can enforce a limit simply because its declaration names a CTRE controller.
        @Override public boolean enforcesSupplyCurrentLimit() {
            var controller = device.kind().controllerKind();
            return controller != null && controller.key().startsWith("ctre:");
        }
        @Override public boolean faulted() { return faulted; }
        @Override public void applyRuntimeConfiguration(MotorRuntimeConfig configuration) { writes.add(configuration); }
        @Override public double appliedVoltage() { return appliedVolts; }
        @Override public double supplyCurrentAmps() { return supplyAmps; }
        @Override public double statorCurrentAmps() { return statorAmps; }
        @Override public double integratedVelocityRotationsPerSecond() { return rotorRps; }
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
    void snapshotIsBuiltOncePerCycleAndRefreshedByTheNextCycle() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 75).currentLimit(Units.Amps.of(400));
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        handle.appliedVolts = 12.0;
        handle.supplyAmps = 364.0;
        RuntimeOverrides overrides = new RuntimeOverrides();
        PowerRuntime power = runtime(6.0, PowerPolicy.defaults().estimateBatteryResistance(false));
        List<MotorDevice> motors = List.of(motor);

        power.cycle(motors, hardware, overrides, 0.0);
        PowerSnapshot first = power.snapshot();
        assertSame(first, power.snapshot(), "telemetry, ARCP and the power publisher share one snapshot per cycle");
        assertEquals(motor.defaultName(), first.motors().get(0).name());
        assertEquals("Power shed", first.motors().get(0).status());

        power.busVoltage(() -> 12.5);
        handle.supplyAmps = 30.0;
        handle.appliedVolts = 1.0;
        power.cycle(motors, hardware, overrides, DT);
        PowerSnapshot second = power.snapshot();
        assertNotSame(first, second, "the next cycle builds a new snapshot");
        assertEquals(12.5, second.busVolts(), 1e-9);
        assertEquals("Declared", second.motors().get(0).status());
        assertEquals(6.0, first.busVolts(), 1e-9, "an earlier snapshot is unchanged");

        PowerPolicy changed = PowerPolicy.defaults().estimateBatteryResistance(false).floor(Units.Volts.of(7.5));
        power.policy(changed);
        assertEquals(7.5, power.snapshot().floorVolts(), 1e-9, "a policy change is visible before the next cycle");
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
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 75).statorCurrentLimit(Units.Amps.of(100))
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

    @Test
    void lowDutyRobotLoadUsesBatterySupplyAmpsAndDoesNotFalselyShedSteering() {
        PowerProfile drive = PowerProfile.of().sustained(Units.Amps.of(60)).share(4.0);
        PowerProfile shooter = PowerProfile.of().sustained(Units.Amps.of(40)).share(2.0);
        List<MotorDevice> motors = new ArrayList<>();
        for (int id = 1; id <= 4; id++) motors.add(MotorDevice.of(MotorKinds.KRAKEN_X60, id)
                .currentLimit(Units.Amps.of(60)).power(drive));
        for (int id = 5; id <= 8; id++) motors.add(MotorDevice.of(MotorKinds.FALCON_500, id)
                .currentLimit(Units.Amps.of(50)));
        for (int id = 31; id <= 34; id++) motors.add(MotorDevice.of(MotorKinds.FALCON_500, id).power(shooter));
        double watts = motors.stream().mapToDouble(motor -> 1.0 / resistance(motor)).sum();
        double bus = (13.0 + Math.sqrt(13.0 * 13.0 - 4.0 * 0.02 * watts)) / 2.0;
        Hardware hardware = new Hardware();
        for (MotorDevice motor : motors) {
            PowerMotor handle = hardware.motor(motor);
            handle.appliedVolts = 1.0;
            handle.statorAmps = 1.0 / resistance(motor);
            handle.supplyAmps = handle.statorAmps / bus;
        }
        PowerRuntime power = runtime(bus, PowerPolicy.defaults().estimateBatteryResistance(false)
                .floor(Units.Volts.of(7.5)));
        power.cycle(motors, hardware, new RuntimeOverrides(), 0.0);

        assertFalse(power.snapshot().shedding(), "293 A winding load is only 23.4 A battery load at 1 V PWM");
        assertEquals(watts / bus, power.snapshot().demandAmps(), 1e-9);
        assertEquals(bus, power.snapshot().predictedVolts(), 1e-9);
        assertEquals(13.0, power.snapshot().openCircuitVolts(), 1e-9);
        assertEquals(Double.POSITIVE_INFINITY, power.capAmps(motors.get(4)), "do not starve steering");
        assertEquals(60.0, power.capAmps(motors.get(0)), "the winding profile still caps torque");
        assertEquals(30.5, power.snapshot().motors().get(0).demandAmps(), 1e-9);
        assertEquals(30.5 / bus, power.snapshot().motors().get(0).supplyDemandAmps(), 1e-9);
    }

    @Test
    void trueBrownoutStillShedsAndConvertedCapBoundsElectricalPower() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 80);
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        handle.appliedVolts = 6.0;
        handle.statorAmps = 6.0 / resistance(motor);
        handle.supplyAmps = handle.statorAmps; // full duty on the 6 V bus
        PowerRuntime power = runtime(6.0, PowerPolicy.defaults().estimateBatteryResistance(false));
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), 0.0);

        assertTrue(power.snapshot().shedding());
        double cap = power.capAmps(motor);
        assertTrue(cap > 0.0 && cap < handle.statorAmps);
        assertEquals(6.0 * power.snapshot().budgetAmps(), resistance(motor) * cap * cap, 1e-8);
        assertEquals("Power shed", power.snapshot().motors().get(0).status());
        assertTrue(handle.writes.isEmpty());
    }

    @Test
    void supplyLimitDoesNotSuppressAnEqualWindingProfileAtPartialDuty() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 81).currentLimit(Units.Amps.of(60))
                .power(PowerProfile.of().sustained(Units.Amps.of(60)));
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        handle.appliedVolts = 1.0;
        handle.statorAmps = 1.0 / resistance(motor);
        handle.supplyAmps = handle.statorAmps / 12.5;
        PowerRuntime power = runtime(12.5, PowerPolicy.defaults());
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), 0.0);
        assertEquals(60.0, power.capAmps(motor), "60 A battery configuration is not a 60 A winding limit");
    }

    @Test
    void activeProfileMustNotBeBypassedJustBecauseItMatchesTheDeclaredStatorLimit() {
        // A vendor option can override the native phase limit above the portable declaration.
        // The independent profile must still reach the host winding clamp.
        MotorDevice motor = MotorDevice.of(MotorKinds.NEO, 86).currentLimit(Units.Amps.of(40))
                .power(PowerProfile.of().sustained(Units.Amps.of(40)));
        Hardware hardware = new Hardware();
        PowerRuntime power = runtime(12.5, PowerPolicy.defaults());
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), 0.0);
        assertEquals(40.0, power.capAmps(motor));
    }

    @Test
    void missingSupplyTelemetryUsesElectricalPowerAndZeroDutyDoesNotInventBatteryLoad() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 82);
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        handle.appliedVolts = 1.0;
        PowerRuntime power = runtime(12.5, PowerPolicy.defaults().estimateBatteryResistance(false));
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), 0.0);
        assertEquals(30.5 / 12.5, power.snapshot().demandAmps(), 1e-9);
        assertEquals(30.5, power.snapshot().motors().get(0).demandAmps(), 1e-9);

        handle.appliedVolts = 0.0;
        handle.rotorRps = 50.0;
        handle.statorAmps = 0.0; // coasting: winding telemetry, rather than a short-circuit model, wins
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), DT);
        assertEquals(0.0, power.snapshot().demandAmps());
        assertEquals(0.0, power.snapshot().motors().get(0).demandAmps());
        assertEquals(Double.POSITIVE_INFINITY, power.capAmps(motor));
    }

    @Test
    void regenerationIsNotCountedAsPositiveLoadOrBorrowedAsAnotherMotorsHeadroom() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 83);
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        double kv = MotorConstants.forKind(motor.kind()).orElseThrow().raw().speedConstantRadPerSecPerVolt();
        handle.rotorRps = 8.0 * kv / (2.0 * Math.PI);
        handle.appliedVolts = 4.0; // back-EMF 8 V: braking/regeneration, despite a magnitude-only vendor reading
        handle.supplyAmps = 20.0;
        handle.statorAmps = 4.0 / resistance(motor);
        PowerRuntime power = runtime(13.0, PowerPolicy.defaults().estimateBatteryResistance(false));
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), 0.0);
        assertEquals(0.0, power.snapshot().demandAmps());
        assertEquals(12.6, power.snapshot().openCircuitVolts(), 1e-9);
        assertEquals(Double.POSITIVE_INFINITY, power.capAmps(motor));
        assertTrue(power.snapshot().motors().get(0).demandAmps() > 0.0, "braking still heats the winding");
    }

    @Test
    void missingBusAndSpeedNeverProduceNaNOutputCaps() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 84)
                .power(PowerProfile.of().sustained(Units.Amps.of(40)));
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        handle.appliedVolts = Double.NaN;
        handle.rotorRps = Double.NaN;
        handle.supplyAmps = 10.0;
        PowerRuntime power = runtime(Double.NaN, PowerPolicy.defaults());
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), 0.0);
        assertEquals(40.0, power.capAmps(motor));
        assertEquals(10.0, power.snapshot().demandAmps());
        assertFalse(Double.isNaN(power.snapshot().motors().get(0).capAmps()));
    }

    @Test
    void unknownMotorConvertsWindingEstimateBackToSupplyForFeedbackClampAndPreservesZero() {
        MotorKind unknown = () -> "test:unknown-power-motor";
        MotorDevice motor = MotorDevice.of(unknown, 85)
                .power(PowerProfile.of().sustained(Units.Amps.of(40)));
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        handle.appliedVolts = 1.0;
        handle.statorAmps = 100.0;
        handle.supplyAmps = 10.0;
        PowerRuntime power = runtime(10.0, PowerPolicy.defaults().estimateBatteryResistance(false));
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), 0.0);
        assertEquals(40.0, power.snapshot().motors().get(0).capAmps());
        assertEquals(4.0, power.capAmps(motor), "unknown-motor output path compares cap to supply telemetry");

        power.busVoltage(() -> 5.0);
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), DT);
        assertTrue(power.snapshot().shedding());
        assertEquals(0.0, power.capAmps(motor), "a zero budget must never become an absent limit");
    }

    @Test
    void thermalDerateUsesWindingReferenceForSupplyOnlyAndProfileOnlyLimitsAndAllowsZero() throws Exception {
        for (int scenario = 0; scenario < 3; scenario++) {
            double fraction = scenario == 2 ? 0.0 : 0.5;
            MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 90 + scenario)
                    .thermal(Thermal.derate(Units.Celsius.of(40), Units.Celsius.of(60), fraction));
            motor = scenario == 1 ? motor.power(PowerProfile.of().sustained(Units.Amps.of(60)))
                    : motor.currentLimit(Units.Amps.of(100));
            Hardware hardware = new Hardware();
            PowerMotor handle = hardware.motor(motor);
            handle.statorAmps = 200.0;
            handle.supplyAmps = 0.0;
            AutoCloseable binding = motor.bindRuntime(new RuntimeScope("power-thermal-domain-" + scenario), handle);
            try {
                PowerRuntime power = runtime(12.5, PowerPolicy.defaults());
                double now = 0.0;
                for (int cycle = 0; cycle < 5000 && motor.temperature().in(Units.Celsius) < 60.0; cycle++) {
                    power.cycle(List.of(motor), hardware, new RuntimeOverrides(), now);
                    now += DT;
                }
                assertTrue(motor.temperature().in(Units.Celsius) >= 60.0);
                power.cycle(List.of(motor), hardware, new RuntimeOverrides(), now);
                double expected = scenario == 1 ? 60.0 * fraction
                        : Math.sqrt(12.5 * 100.0 / resistance(motor)) * fraction;
                assertEquals(expected, power.capAmps(motor), 1e-8,
                        "thermal fractions scale winding amps, not an equal numeric supply limit");
                assertEquals("Thermal derate", power.snapshot().motors().get(0).status());
            } finally {
                binding.close();
            }
        }
    }

    @Test
    void revExplicitSupplyLimitIsConvertedAndEnforcedWhileCoolOnAHealthyBus() {
        MotorDevice motor = MotorDevice.of(MotorKinds.NEO, 94)
                .supplyCurrentLimit(Units.Amps.of(5)).statorCurrentLimit(Units.Amps.of(80));
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        handle.appliedVolts = 1.0;
        handle.statorAmps = 1.0 / resistance(motor);
        handle.supplyAmps = handle.statorAmps / 12.0;
        PowerRuntime power = runtime(12.0, PowerPolicy.defaults().estimateBatteryResistance(false));
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), 0.0);
        double cap = power.capAmps(motor);
        assertFalse(power.snapshot().shedding());
        assertEquals(Math.sqrt(12.0 * 5.0 / resistance(motor)), cap, 1e-9);
        assertTrue(cap > 5.0 && cap < 80.0, "supply amps cannot be substituted numerically for winding amps");
        assertEquals(1.0, power.snapshot().motors().get(0).derateFraction());
        assertTrue(handle.writes.isEmpty(), "software supply enforcement never reconfigures SPARK in the loop");
    }

    @Test
    void unknownRevMotorUsesExplicitlyReportedSupplyFeedbackEstimate() {
        MotorKind unknown = () -> "test:unknown-rev-motor";
        MotorDevice motor = MotorDevice.of(unknown.controlledBy(MotorControllerKinds.SPARK_MAX), 95)
                .supplyCurrentLimit(Units.Amps.of(5));
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        handle.appliedVolts = 1.0;
        handle.statorAmps = 100.0;
        handle.supplyAmps = 10.0;
        PowerRuntime power = runtime(12.0, PowerPolicy.defaults().estimateBatteryResistance(false));
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), 0.0);
        assertEquals(5.0, power.capAmps(motor));
        assertEquals("Supply limit estimated", power.snapshot().motors().get(0).status());
    }

    @Test
    void explicitRevSupplyLimitDoesNotInventVoltageOrSpeedWhenTelemetryIsMissing() {
        MotorDevice motor = MotorDevice.of(MotorKinds.NEO, 96).supplyCurrentLimit(Units.Amps.of(5));
        Hardware hardware = new Hardware();
        PowerMotor handle = hardware.motor(motor);
        handle.supplyAmps = 0.0;
        handle.statorAmps = 0.0;
        PowerRuntime power = runtime(Double.NaN, PowerPolicy.defaults());
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), 0.0);
        assertEquals(0.0, power.capAmps(motor), "no terminal/bus voltage means no known positive power allowance");
        assertEquals("Supply limit unavailable", power.snapshot().motors().get(0).status());

        handle.appliedVolts = 1.0;
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), DT);
        assertEquals(Math.sqrt(1.0 * 5.0 / resistance(motor)), power.capAmps(motor), 1e-9,
                "terminal voltage gives a conservative lower bound without assuming the battery floor");

        power.busVoltage(() -> 12.0);
        handle.rotorRps = Double.NaN;
        power.cycle(List.of(motor), hardware, new RuntimeOverrides(), 2 * DT);
        assertEquals(0.0, power.capAmps(motor), "do not guess back-EMF when winding speed is missing");
        assertEquals("Supply limit unavailable", power.snapshot().motors().get(0).status());
    }

    @Test
    void ctreDeclaredLeaderUsesSoftwareSupplyLimitAfterPromotingARevController() {
        // Same physical motor on both controllers isolates routing from differences in motor constants.
        MotorDevice leader = MotorDevice.of(MotorKinds.NEO.controlledBy(MotorControllerKinds.TALON_FXS), 97)
                .supplyCurrentLimit(Units.Amps.of(5)).statorCurrentLimit(Units.Amps.of(80));
        MotorDevice survivor = MotorDevice.of(MotorKinds.NEO, 98)
                .supplyCurrentLimit(Units.Amps.of(5)).statorCurrentLimit(Units.Amps.of(80)).follow(leader);
        Map<Integer, PowerMotor> backends = new HashMap<>();
        MotorBackend backend = new MotorBackend() {
            @Override public boolean supports(MotorKind kind) { return true; }
            @Override public MotorHandle create(MotorDevice device) {
                PowerMotor handle = new PowerMotor(device);
                handle.appliedVolts = 1.0;
                handle.statorAmps = 1.0 / resistance(device);
                handle.supplyAmps = handle.statorAmps / 12.0;
                backends.put(device.id(), handle);
                return handle;
            }
        };
        try (HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(backend))) {
            var group = graph.motorGroup(new MotorGroupPolicy(
                    leader, List.of(survivor), true, true, 1, 1.0, 1.0, true, 1));
            MotorHandle route = graph.motor(leader);
            graph.motor(survivor);
            graph.refreshInputs();
            PowerRuntime power = runtime(12.0, PowerPolicy.defaults().estimateBatteryResistance(false));
            RuntimeOverrides overrides = new RuntimeOverrides();
            power.cycle(List.of(leader), graph, overrides, 0.0);
            assertTrue(route.enforcesSupplyCurrentLimit());
            assertEquals(Double.POSITIVE_INFINITY, power.capAmps(leader));

            backends.get(leader.id()).faulted = true;
            graph.refreshInputs();
            assertSame(survivor, group.leader());
            assertSame(route, graph.motor(leader), "the cached wrapper survives promotion");
            assertFalse(route.enforcesSupplyCurrentLimit(), "the active REV route needs host supply enforcement");
            power.cycle(List.of(leader), graph, overrides, DT);
            assertEquals(Math.sqrt(12.0 * 5.0 / resistance(survivor)), power.capAmps(leader), 1e-9);
        }
    }

    @Test
    void customHandleDefaultsToSoftwareEnforcementEvenForACtreDeclaration() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 99).supplyCurrentLimit(Units.Amps.of(5));
        MotorHandle custom = new MotorHandle() {
            @Override public MotorDevice device() { return motor; }
            @Override public double appliedVoltage() { return 0.0; }
            @Override public double supplyCurrentAmps() { return 0.0; }
            @Override public double statorCurrentAmps() { return 0.0; }
            @Override public double integratedVelocityRotationsPerSecond() { return 0.0; }
        };
        PowerRuntime power = runtime(12.0, PowerPolicy.defaults().estimateBatteryResistance(false));
        power.cycle(List.of(motor), new ActionContext() {
            @Override public MotorHandle motor(MotorDevice device) { return custom; }
        }, new RuntimeOverrides(), 0.0);
        assertFalse(custom.enforcesSupplyCurrentLimit());
        assertEquals(Math.sqrt(12.0 * 5.0 / resistance(motor)), power.capAmps(motor), 1e-9);
    }

    private static double resistance(MotorDevice motor) {
        return MotorConstants.forKind(motor.kind()).orElseThrow().resistance().in(Units.Ohms);
    }
}
