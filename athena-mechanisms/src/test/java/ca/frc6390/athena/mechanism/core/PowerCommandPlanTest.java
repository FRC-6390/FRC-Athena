package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.power.PowerPolicy;
import ca.frc6390.athena.hardware.power.PowerProfile;
import ca.frc6390.athena.hardware.power.Thermal;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.RuntimeScope;
import edu.wpi.first.units.Units;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;

class PowerCommandPlanTest {
    private static final double BUS = 12.0;
    private static final double RESISTANCE = MotorConstants.forKind(MotorKinds.KRAKEN_X60)
            .orElseThrow().resistance().in(Units.Ohms);
    private static final double SPEED_CONSTANT = MotorConstants.forKind(MotorKinds.KRAKEN_X60)
            .orElseThrow().raw().speedConstantRadPerSecPerVolt();

    private static final class Motor implements MotorHandle {
        final MotorDevice device;
        MotorHandle target;
        double backEmf;
        double commandScale = 1.0;
        double measuredWinding;
        double measuredTemperature = Double.NaN;
        boolean missingSpeed;
        boolean nativeSupply = true;
        Motor(MotorDevice device) { this.device = device; }
        @Override public MotorDevice device() { return device; }
        @Override public MotorHandle commandTarget() { return target == null ? this : target; }
        @Override public double voltageCommandScale() { return commandScale; }
        @Override public double integratedVelocityRotationsPerSecond() {
            return missingSpeed ? Double.NaN : backEmf * SPEED_CONSTANT / (2.0 * Math.PI);
        }
        @Override public double appliedVoltage() { return 0.0; }
        @Override public double supplyCurrentAmps() { return 0.0; }
        @Override public double statorCurrentAmps() { return measuredWinding; }
        @Override public boolean supportsRuntimeConfiguration() { return true; }
        @Override public boolean enforcesSupplyCurrentLimit() { return nativeSupply; }
        @Override public OptionalDouble temperatureCelsius() {
            return Double.isFinite(measuredTemperature) ? OptionalDouble.of(measuredTemperature) : OptionalDouble.empty();
        }
        @Override public void applyRuntimeConfiguration(MotorRuntimeConfig configuration) {}
    }

    private static final class Hardware implements ActionContext {
        final Map<MotorDevice, Motor> handles = new IdentityHashMap<>();
        @Override public Motor motor(MotorDevice device) { return handles.computeIfAbsent(device, Motor::new); }
    }

    private static PowerRuntime power() {
        PowerRuntime power = new PowerRuntime();
        power.policy(PowerPolicy.defaults().estimateBatteryResistance(false));
        power.busVoltage(() -> BUS);
        return power;
    }

    private static RuntimeOverrides overrides(Hardware hardware) {
        RuntimeOverrides overrides = new RuntimeOverrides();
        overrides.actionContext(hardware);
        return overrides;
    }

    @Test
    void physicalAliasesCountOnceWhileEveryDeclarationStillConstrainsTheCommand() {
        MotorDevice owner = MotorDevice.of(MotorKinds.KRAKEN_X60, 1).power(PowerProfile.of().share(4.0));
        MotorDevice alias = MotorDevice.of(MotorKinds.KRAKEN_X60, 2).power(PowerProfile.of().share(4.0));
        MotorDevice other = MotorDevice.of(MotorKinds.KRAKEN_X60, 3).power(PowerProfile.of());
        Hardware hardware = new Hardware();
        hardware.motor(alias).target = hardware.motor(owner);
        RuntimeOverrides overrides = overrides(hardware);
        PowerRuntime power = power();
        Object family = new Object();
        Object unrelated = new Object();
        List<MotorDevice> motors = List.of(owner, alias, other);
        MotorHandle[] familyHandles = {hardware.motor(owner), hardware.motor(alias)};

        power.beginCycle(motors, hardware, overrides, 0.0);
        power.registerCommand(family, List.of(owner, alias), familyHandles, 6.0,
                new double[] {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY});
        power.registerCommand(unrelated, List.of(other), new MotorHandle[] {hardware.motor(other)}, 6.0,
                new double[] {Double.POSITIVE_INFINITY});
        power.planCommands();

        assertFalse(power.commandSuppressed(family));
        assertFalse(power.snapshot().shedding(), "two physical loads fit; counting the alias would breach the floor");
        assertEquals(2.0 * 36.0 / (BUS * RESISTANCE), power.snapshot().demandAmps(), 1e-9);
        assertEquals(6.0, power.plannedVoltage(family, Double.NaN), 1e-9);
        assertTrue(power.claimCommandTarget(hardware.motor(owner)));
        assertFalse(power.claimCommandTarget(hardware.motor(alias)));

        overrides.statorLimit(alias, 50.0);
        power.beginCycle(motors, hardware, overrides, 0.02);
        power.registerCommand(family, List.of(owner, alias), familyHandles, 6.0,
                new double[] {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY});
        power.planCommands();

        assertFalse(power.commandSuppressed(family));
        assertEquals(50.0 * RESISTANCE, power.plannedVoltage(family, Double.NaN), 1e-9,
                "the alias cap participates even though its battery demand and share were deduplicated");
    }

    @Test
    void removingAnInfeasibleSupplyLimitRecoversTheSamePreparedCommand() {
        MotorDevice root = MotorDevice.of(MotorKinds.KRAKEN_X60, 4).statorCurrentLimit(Units.Amps.of(10));
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 5)
                .statorCurrentLimit(Units.Amps.of(400)).supplyCurrentLimit(Units.Amps.of(1));
        Hardware hardware = new Hardware();
        hardware.motor(root).backEmf = 8.0;
        RuntimeOverrides overrides = overrides(hardware);
        PowerRuntime power = power();
        Object command = new Object();
        List<MotorDevice> motors = List.of(root, follower);
        MotorHandle[] handles = {hardware.motor(root), hardware.motor(follower)};
        double[] local = {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY};

        power.beginCycle(motors, hardware, overrides, 0.0);
        power.registerCommand(command, motors, handles, 10.0, local);
        power.planCommands();
        assertTrue(power.commandSuppressed(command), "root current cap requires volts above the follower supply limit");

        overrides.supplyLimit(follower, 0.0);
        power.beginCycle(motors, hardware, overrides, 0.02);
        power.registerCommand(command, motors, handles, 10.0, local);
        power.planCommands();

        assertFalse(power.commandSuppressed(command), "the old synthetic supply constraint must not remain infeasible");
        double volts = power.plannedVoltage(command, Double.NaN);
        assertEquals(8.0 + 10.0 * RESISTANCE, volts, 1e-9);
        assertTrue(Math.abs((volts - 8.0) / RESISTANCE) <= 10.0 + 1e-9);
        assertTrue(Math.abs(volts / RESISTANCE) <= 400.0);
    }

    @Test
    void invertedFollowerAtDifferentSpeedConstrainsOneSharedRootVoltage() {
        MotorDevice root = MotorDevice.of(MotorKinds.KRAKEN_X60, 6).statorCurrentLimit(Units.Amps.of(100));
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 7).statorCurrentLimit(Units.Amps.of(100));
        Hardware hardware = new Hardware();
        hardware.motor(root).backEmf = 4.0;
        hardware.motor(follower).backEmf = -2.0;
        hardware.motor(follower).commandScale = -1.0;
        RuntimeOverrides overrides = overrides(hardware);
        PowerRuntime power = power();
        Object command = new Object();
        List<MotorDevice> motors = List.of(root, follower);

        power.beginCycle(motors, hardware, overrides, 0.0);
        power.registerCommand(command, motors,
                new MotorHandle[] {hardware.motor(root), hardware.motor(follower)}, 10.0,
                new double[] {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY});
        power.planCommands();

        assertFalse(power.commandSuppressed(command));
        double volts = power.plannedVoltage(command, Double.NaN);
        assertEquals(2.0 + 100.0 * RESISTANCE, volts, 1e-9);
        assertTrue(Math.abs((volts - 4.0) / RESISTANCE) <= 100.0 + 1e-9);
        assertEquals(-100.0, (-volts + 2.0) / RESISTANCE, 1e-9);
    }

    @Test
    void followerSpinningAgainstItsInvertedCommandReportsIncompatibleCurrentIntervals() {
        MotorDevice root = MotorDevice.of(MotorKinds.KRAKEN_X60, 8).statorCurrentLimit(Units.Amps.of(100));
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 9).statorCurrentLimit(Units.Amps.of(100));
        Hardware hardware = new Hardware();
        hardware.motor(root).backEmf = 4.0;
        hardware.motor(follower).backEmf = 8.0;
        hardware.motor(follower).commandScale = -1.0;
        RuntimeOverrides overrides = overrides(hardware);
        PowerRuntime power = power();
        Object command = new Object();
        List<MotorDevice> motors = List.of(root, follower);

        power.beginCycle(motors, hardware, overrides, 0.0);
        power.registerCommand(command, motors,
                new MotorHandle[] {hardware.motor(root), hardware.motor(follower)}, 10.0,
                new double[] {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY});
        power.planCommands();

        assertTrue(power.commandSuppressed(command));
        assertEquals(0.0, power.capAmps(root));
        assertEquals(0.0, power.capAmps(follower));
        assertTrue(power.snapshot().motors().stream().allMatch(motor -> motor.status().contains("Infeasible")));
    }

    @Test
    void missingSpeedCannotBypassASoftwareSupplyLimitUsingPreviousWindingCurrent() {
        MotorDevice device = MotorDevice.of(MotorKinds.KRAKEN_X60, 10)
                .supplyCurrentLimit(Units.Amps.of(10));
        Hardware hardware = new Hardware();
        Motor motor = hardware.motor(device);
        motor.nativeSupply = false;
        motor.missingSpeed = true;
        motor.measuredWinding = 50.0;
        PowerRuntime power = power();
        Object command = new Object();

        power.beginCycle(List.of(device), hardware, overrides(hardware), 0.0);
        power.registerCommand(command, List.of(device), new MotorHandle[] {motor}, 6.0,
                new double[] {Double.POSITIVE_INFINITY});
        power.planCommands();

        assertTrue(power.commandSuppressed(command), "previous 50 A winding current cannot certify a new 10 A supply limit");
        assertEquals(0.0, power.capAmps(device));
        assertEquals(0.0, power.snapshot().demandAmps());
        assertTrue(power.snapshot().motors().get(0).status().contains("Supply limit unavailable"));
    }

    @Test
    void neutralAndScalarAliasesAreBothSuppressedRegardlessOfRegistrationOrder() {
        for (boolean neutralFirst : List.of(false, true)) {
            MotorDevice owner = MotorDevice.of(MotorKinds.KRAKEN_X60, 11);
            MotorDevice alias = MotorDevice.of(MotorKinds.KRAKEN_X60, 12);
            Hardware hardware = new Hardware();
            hardware.motor(alias).target = hardware.motor(owner);
            PowerRuntime power = power();
            Object scalar = new Object();
            Object neutral = new Object();
            power.beginCycle(List.of(owner, alias), hardware, overrides(hardware), 0.0);
            if (neutralFirst) registerNeutral(power, neutral, alias, hardware.motor(alias));
            power.registerCommand(scalar, List.of(owner), new MotorHandle[] {hardware.motor(owner)}, 6.0,
                    new double[] {Double.POSITIVE_INFINITY});
            if (!neutralFirst) registerNeutral(power, neutral, alias, hardware.motor(alias));
            power.planCommands();

            assertTrue(power.commandSuppressed(scalar));
            assertTrue(power.commandSuppressed(neutral));
            assertEquals(0.0, power.capAmps(owner));
            assertEquals(0.0, power.capAmps(alias));
            assertEquals(0.0, power.snapshot().demandAmps());
            assertTrue(power.snapshot().motors().stream()
                    .allMatch(motor -> motor.status().contains("Conflicting physical command targets")));
        }
    }

    private static void registerNeutral(PowerRuntime power, Object key, MotorDevice device, Motor motor) {
        power.registerCommand(key, List.of(device), new MotorHandle[] {motor}, Double.NaN,
                new double[] {Double.POSITIVE_INFINITY});
    }

    @Test
    void disabledFollowerDoesNotStopOrConstrainItsActiveCommandFamily() {
        PowerProfile profile = PowerProfile.of();
        MotorDevice root = MotorDevice.of(MotorKinds.KRAKEN_X60, 13)
                .statorCurrentLimit(Units.Amps.of(100)).power(profile);
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 14)
                .statorCurrentLimit(Units.Amps.of(1)).disabled(true).power(profile);
        Hardware hardware = new Hardware();
        PowerRuntime power = power();
        Object command = new Object();
        List<MotorDevice> motors = List.of(root, follower);

        power.beginCycle(motors, hardware, overrides(hardware), 0.0);
        power.registerCommand(command, motors,
                new MotorHandle[] {hardware.motor(root), hardware.motor(follower)}, 6.0,
                new double[] {Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY});
        power.planCommands();

        assertFalse(power.commandSuppressed(command));
        assertEquals(100.0 * RESISTANCE, power.plannedVoltage(command, Double.NaN), 1e-9);
        assertEquals(1, power.snapshot().motors().size());
    }

    @Test
    @SuppressWarnings("try") // The binding's lifetime supplies the thermal runtime to the declaration.
    void hotSupplyOnlyMotorDeratesTheConvertedSupplyCurrentReferenceIncludingLateRegistration() throws Exception {
        for (boolean registeredLate : List.of(false, true)) {
            MotorDevice device = MotorDevice.of(MotorKinds.KRAKEN_X60, registeredLate ? 16 : 15)
                    .supplyCurrentLimit(Units.Amps.of(10))
                    .thermal(Thermal.derate(Units.Celsius.of(40), Units.Celsius.of(60), 0.5));
            Hardware hardware = new Hardware();
            Motor motor = hardware.motor(device);
            motor.measuredTemperature = 60.0;
            try (AutoCloseable binding = device.bindRuntime(
                    new RuntimeScope("candidate-supply-thermal-" + registeredLate), motor)) {
                device.thermalModel().vendorReading(60.0);
                PowerRuntime power = power();
                Object command = new Object();
                power.beginCycle(registeredLate ? List.of() : List.of(device), hardware, overrides(hardware), 0.0);
                power.registerCommand(command, List.of(device), new MotorHandle[] {motor}, 6.0,
                        new double[] {Double.POSITIVE_INFINITY});
                power.planCommands();

                double expectedCurrent = 0.5 * Math.sqrt(BUS * 10.0 / RESISTANCE);
                assertFalse(power.commandSuppressed(command));
                assertEquals(expectedCurrent * RESISTANCE, power.plannedVoltage(command, Double.NaN), 1e-9,
                        "thermal winding reference must come from the declared supply cap, not stall current");
                assertEquals(expectedCurrent, power.capAmps(device), 1e-9);
                assertEquals(0.5, power.snapshot().motors().get(0).derateFraction());
            }
        }
    }
}
