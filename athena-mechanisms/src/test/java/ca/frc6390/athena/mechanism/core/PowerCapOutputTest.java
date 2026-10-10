package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.power.PowerPolicy;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.OptionalDouble;
import org.junit.jupiter.api.Test;

/** The power cap reaches every motor command on the output path (B6), and no configuration is written. */
class PowerCapOutputTest {
    private static final MotorConstants KRAKEN = MotorConstants.forKind(MotorKinds.KRAKEN_X60).orElseThrow();

    /** Motor handle with settable speed and supply current; records voltage, percent and any configuration write. */
    private static final class Motor implements MotorHandle {
        final MotorDevice device;
        double rotationsPerSecond;
        double supplyAmps;
        double volts = Double.NaN;
        double percent = Double.NaN;
        final List<MotorRuntimeConfig> writes = new ArrayList<>();

        Motor(MotorDevice device) { this.device = device; }

        @Override public MotorDevice device() { return device; }
        @Override public boolean supportsRuntimeConfiguration() { return true; }
        @Override public void applyRuntimeConfiguration(MotorRuntimeConfig configuration) { writes.add(configuration); }
        @Override public double appliedVoltage() { return volts; }
        @Override public void setVoltage(double value) { volts = value; }
        @Override public void setPercentOutput(double value) { percent = value; }
        @Override public double supplyCurrentAmps() { return supplyAmps; }
        @Override public double statorCurrentAmps() { return supplyAmps; }
        @Override public double integratedVelocityRotationsPerSecond() { return rotationsPerSecond; }
        @Override public OptionalDouble temperatureCelsius() { return OptionalDouble.empty(); }
    }

    /** Hardware with an optional software follower of a leader. */
    private static final class Hardware implements ActionContext {
        final Map<MotorDevice, Motor> motors = new HashMap<>();
        MotorDevice leader;
        MotorDevice follower;

        @Override public Motor motor(MotorDevice motor) {
            return motors.computeIfAbsent(motor, Motor::new);
        }

        @Override public List<SoftwareMotorFollower> softwareFollowers(MotorDevice device) {
            return device.equals(leader) && follower != null
                    ? List.of(new SoftwareMotorFollower(follower, motor(follower)))
                    : List.of();
        }
    }

    /** Predicted stator current of a voltage at a speed, the quantity the clamp bounds. */
    private static double currentAt(double volts, double rotationsPerSecond) {
        double omega = rotationsPerSecond * 2.0 * Math.PI;
        return (volts - omega / KRAKEN.speedConstant()) / KRAKEN.resistanceOhms();
    }

    /** Runs the power pass on the motors, then applies one output through the applier with the pass installed. */
    private static final class Rig {
        final Hardware hardware = new Hardware();
        final PowerRuntime power = new PowerRuntime();
        final OutputApplier applier;
        final List<MotorDevice> motors;

        Rig(MotorDevice... devices) {
            power.policy(PowerPolicy.defaults().estimateBatteryResistance(false));
            power.busVoltage(() -> 12.5);
            motors = List.of(devices);
            applier = OutputApplier.using(hardware);
            applier.electrical(power);
        }

        void cycle(double now) {
            power.cycle(motors, hardware, new RuntimeOverrides(), now);
        }

        void apply(MotorDevice motor, Output output) {
            applier.apply(new ResolvedOutput(OutputRequest.of(motor, output), output));
        }
    }

    @Test
    void stalledMotorPercentOutputIsCappedToItsCurrentLimit() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 90).currentLimit(20);
        Rig rig = new Rig(motor);
        rig.cycle(0.0);
        assertEquals(20.0, rig.power.capAmps(motor), 1e-9);

        rig.apply(motor, Outputs.percent(1.0));

        Motor handle = rig.hardware.motor(motor);
        assertTrue(Double.isNaN(handle.percent), "a capped percent command is sent as voltage");
        assertEquals(20.0 * KRAKEN.resistanceOhms(), handle.volts, 1e-9);
        assertTrue(currentAt(handle.volts, 0.0) <= 20.0 + 1e-9, "stalled current within the cap");
        assertTrue(handle.writes.isEmpty(), "no configuration write");
    }

    @Test
    void movingMotorVoltageIsClampedAroundItsBackEmf() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 91).currentLimit(20);
        Rig rig = new Rig(motor);
        Motor handle = rig.hardware.motor(motor);
        // Back-EMF of 6 V: the command of 12 V would drive about (12 - 6) / R amps, far above the cap.
        handle.rotationsPerSecond = 6.0 * KRAKEN.speedConstant() / (2.0 * Math.PI);
        rig.cycle(0.0);

        rig.apply(motor, Outputs.voltage(12.0));

        assertEquals(6.0 + 20.0 * KRAKEN.resistanceOhms(), handle.volts, 1e-9);
        assertTrue(currentAt(handle.volts, handle.rotationsPerSecond) <= 20.0 + 1e-9, "moving current within the cap");
    }

    @Test
    void motorWithoutConstantsIsScaledByItsMeasuredSupplyCurrent() {
        MotorDevice minion = MotorDevice.of(MotorKinds.MINION, 92).currentLimit(20);
        Rig rig = new Rig(minion);
        Motor handle = rig.hardware.motor(minion);
        handle.supplyAmps = 40.0;
        rig.cycle(0.0);

        rig.apply(minion, Outputs.voltage(12.0));

        assertEquals(6.0, handle.volts, 1e-9);
        assertTrue(handle.writes.isEmpty(), "no configuration write");
    }

    @Test
    void softwareFollowerCapBoundsTheSharedCommand() {
        MotorDevice leader = MotorDevice.of(MotorKinds.KRAKEN_X60, 93).currentLimit(400);
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 94).currentLimit(5);
        Rig rig = new Rig(leader, follower);
        rig.hardware.leader = leader;
        rig.hardware.follower = follower;
        rig.cycle(0.0);

        rig.apply(leader, Outputs.voltage(12.0));

        double expected = 5.0 * KRAKEN.resistanceOhms();
        assertEquals(expected, rig.hardware.motor(leader).volts, 1e-9);
        assertEquals(expected, rig.hardware.motor(follower).volts, 1e-9);
    }
}
