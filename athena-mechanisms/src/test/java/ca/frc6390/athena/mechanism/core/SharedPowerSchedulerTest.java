package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorConstants;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.power.PowerPolicy;
import ca.frc6390.athena.hardware.power.PowerProfile;
import ca.frc6390.athena.hardware.power.Thermal;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.effort.Effort;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import edu.wpi.first.units.Units;
import java.util.IdentityHashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

/** Exercises the public scheduler across independently registered mechanisms, starting from zero current. */
class SharedPowerSchedulerTest {
    private static final class Motor implements MotorHandle {
        final MotorDevice device;
        final AtomicInteger allCalculations;
        double speed;
        double position;
        double volts;
        int writes;
        Motor(MotorDevice device, AtomicInteger allCalculations) {
            this.device = device;
            this.allCalculations = allCalculations;
        }
        @Override public MotorDevice device() { return device; }
        @Override public double integratedPositionRotations() { return position; }
        @Override public double integratedVelocityRotationsPerSecond() { return speed; }
        @Override public double appliedVoltage() { return volts; }
        @Override public double statorCurrentAmps() { return 0.0; }
        @Override public double supplyCurrentAmps() { return 0.0; }
        @Override public void setVoltage(double value) {
            assertEquals(2, allCalculations.get(), "every mechanism must be prepared before the first write");
            volts = value;
            writes++;
        }
        @Override public void stop() { volts = 0.0; }
    }

    private static final class Loop implements ControlLoop {
        final double requestedVolts;
        final AtomicInteger allCalculations;
        int calculations;
        int notifications;
        double appliedVolts;
        Loop(double requestedVolts, AtomicInteger allCalculations) {
            this.requestedVolts = requestedVolts;
            this.allCalculations = allCalculations;
        }
        @Override public ControlLoopRuntime bind(ControlLoopBinding binding) {
            return new ControlLoopRuntime() {
                @Override public ControlOutput calculate(ControlLoopContext context) {
                    calculations++;
                    allCalculations.incrementAndGet();
                    return ControlOutput.voltage(requestedVolts);
                }
                @Override public void applied(ControlLoopContext context, Output requested, Output applied) {
                    notifications++;
                    appliedVolts = applied instanceof Output.Voltage voltage ? voltage.volts() : Double.NaN;
                }
            };
        }
    }

    private static final class Wheel implements Mechanism {
        final MotorDevice motor;
        final PowerPolicy policy;
        final ControlBinding<?> velocity;
        final Action drive;
        Wheel(MotorDevice motor, Loop loop, PowerPolicy policy, double localLimit) {
            this.motor = motor;
            this.policy = policy;
            ControlBinding<?> binding = Controls.velocity(motor, Units.Rotations).loop(loop);
            velocity = Double.isFinite(localLimit) ? binding.effort(Effort.max(Units.Amps.of(localLimit))) : binding;
            drive = velocity.velocity(Units.RotationsPerSecond.of(10.0));
        }
    }

    private static final class GuardedWheel implements Mechanism {
        final MotorDevice motor;
        final PowerPolicy policy;
        final ControlBinding<?> position;
        final Action drive;
        GuardedWheel(MotorDevice motor, Loop loop, PowerPolicy policy) {
            this.motor = motor;
            this.policy = policy;
            position = Controls.position(motor, Units.Rotations).loop(loop)
                    .constraint(Constraints.range(Range.of(Units.Rotations.of(0.0), Units.Rotations.of(1.0))));
            drive = position.position(Units.Rotations.of(0.0));
        }
    }

    private static final class Rig {
        final AtomicInteger calculations = new AtomicInteger();
        final Loop firstLoop = new Loop(12.0, calculations);
        final Loop secondLoop = new Loop(8.0, calculations);
        final Motor first;
        final Motor second;
        final MechanismScheduler scheduler;
        Rig(PowerProfile firstProfile, PowerProfile secondProfile, double firstLocalLimit, boolean hotFirst) {
            MotorDevice firstDevice = MotorDevice.of(MotorKinds.KRAKEN_X60, 211).power(firstProfile);
            if (hotFirst) {
                // Ambient 25 C lies above this scripted curve, giving a deterministic 25% thermal limit.
                firstDevice = firstDevice.thermal(Thermal.derate(Units.Celsius.of(10.0), Units.Celsius.of(20.0), 0.25));
            }
            first = new Motor(firstDevice, calculations);
            second = new Motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 212).power(secondProfile), calculations);
            second.speed = 40.0;
            Map<MotorDevice, Motor> motors = new IdentityHashMap<>();
            motors.put(first.device, first);
            motors.put(second.device, second);
            PowerPolicy policy = PowerPolicy.defaults().estimateBatteryResistance(false)
                    .staggerStarts(Units.Seconds.of(0.0));
            Wheel firstWheel = new Wheel(first.device, firstLoop, policy, firstLocalLimit);
            Wheel secondWheel = new Wheel(second.device, secondLoop, policy, Double.POSITIVE_INFINITY);
            scheduler = MechanismScheduler.create(new ActionContext() {
                @Override public MotorHandle motor(MotorDevice device) { return motors.get(device); }
            }).register(firstWheel).register(secondWheel).busVoltage(() -> 12.0);
            scheduler.request(firstWheel.drive);
            scheduler.request(secondWheel.drive);
        }
        void firstCycle() { scheduler.teleopPeriodic(0.0, 0.02); }
        double firstFactor() { return current(first, first.volts) / current(first, firstLoop.requestedVolts); }
        double secondFactor() { return current(second, second.volts) / current(second, secondLoop.requestedVolts); }
        void assertOneCalculationAndWriteEach() {
            assertEquals(1, firstLoop.calculations);
            assertEquals(1, secondLoop.calculations);
            assertEquals(1, first.writes);
            assertEquals(1, second.writes);
            assertEquals(1, firstLoop.notifications);
            assertEquals(1, secondLoop.notifications);
            assertEquals(first.volts, firstLoop.appliedVolts, 1e-12);
            assertEquals(second.volts, secondLoop.appliedVolts, 1e-12);
        }
    }

    @Test
    void sharedProfileCouplesTheFirstOutputAcrossMechanismsWithDifferentSpeeds() {
        PowerProfile profile = PowerProfile.of().sustained(Units.Amps.of(40.0));
        Rig rig = new Rig(profile, profile, Double.POSITIVE_INFINITY, false);

        rig.firstCycle();

        assertEquals(rig.firstFactor(), rig.secondFactor(), 1e-8);
        assertEquals(40.0, current(rig.first, rig.first.volts), 1e-7);
        assertTrue(current(rig.second, rig.second.volts) > 0.0);
        assertTrue(current(rig.second, rig.second.volts) < 40.0);
        assertTrue(rig.scheduler.powerSnapshot().motors().stream().allMatch(motor -> motor.demandAmps() > 50.0),
                "first-cycle planning uses requested output despite zero measured current");
        rig.assertOneCalculationAndWriteEach();
    }

    @Test
    void copiedProfileSettingsRemainIndependentAcrossMechanisms() {
        PowerProfile profile = PowerProfile.of().sustained(Units.Amps.of(40.0));
        Rig rig = new Rig(profile, profile.copy(), Double.POSITIVE_INFINITY, false);

        rig.firstCycle();

        assertEquals(40.0, current(rig.first, rig.first.volts), 1e-7);
        assertEquals(40.0, current(rig.second, rig.second.volts), 1e-7);
        assertTrue(Math.abs(rig.firstFactor() - rig.secondFactor()) > 0.1,
                "equal settings in separate profile instances must not couple their current factors");
        rig.assertOneCalculationAndWriteEach();
    }

    @Test
    void oneWheelLocalEffortCeilingReducesTheWholeSharedGroupProportionally() {
        PowerProfile profile = PowerProfile.of().sustained(Units.Amps.of(80.0));
        Rig rig = new Rig(profile, profile, 10.0, false);

        rig.firstCycle();

        assertEquals(10.0, current(rig.first, rig.first.volts), 1e-7);
        assertEquals(rig.firstFactor(), rig.secondFactor(), 1e-8);
        assertTrue(current(rig.second, rig.second.volts) > 0.0);
        rig.assertOneCalculationAndWriteEach();
    }

    @Test
    void oneWheelThermalDerateReducesTheWholeSharedGroupProportionally() {
        PowerProfile profile = PowerProfile.of().sustained(Units.Amps.of(80.0));
        Rig rig = new Rig(profile, profile, Double.POSITIVE_INFINITY, true);

        rig.firstCycle();

        assertEquals(20.0, current(rig.first, rig.first.volts), 1e-7);
        assertEquals(rig.firstFactor(), rig.secondFactor(), 1e-8);
        assertTrue(rig.scheduler.powerSnapshot().motors().stream().anyMatch(motor -> motor.derateFraction() == 0.25));
        rig.assertOneCalculationAndWriteEach();
    }

    @Test
    void allocatedDirectionGuardNeutralizesTheWholeGroupBeforeAnyPeerWrites() {
        PowerProfile profile = PowerProfile.of().sustained(Units.Amps.of(10.0));
        PowerPolicy policy = PowerPolicy.defaults().estimateBatteryResistance(false)
                .staggerStarts(Units.Seconds.of(0.0));
        AtomicInteger calculations = new AtomicInteger();
        Motor peer = new Motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 213).power(profile), calculations);
        Motor guarded = new Motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 214).power(profile), calculations);
        guarded.position = 1.0;
        guarded.speed = 4.0 * MotorConstants.forKind(guarded.device.kind()).orElseThrow()
                .raw().speedConstantRadPerSecPerVolt() / (2.0 * Math.PI);
        Loop peerLoop = new Loop(6.0, calculations);
        Loop guardedLoop = new Loop(-6.0, calculations);
        Wheel peerWheel = new Wheel(peer.device, peerLoop, policy, Double.POSITIVE_INFINITY);
        GuardedWheel guardedWheel = new GuardedWheel(guarded.device, guardedLoop, policy);
        Map<MotorDevice, Motor> motors = new IdentityHashMap<>();
        motors.put(peer.device, peer);
        motors.put(guarded.device, guarded);
        MechanismScheduler scheduler = MechanismScheduler.create(new ActionContext() {
            @Override public MotorHandle motor(MotorDevice motor) { return motors.get(motor); }
        }).register(peerWheel).register(guardedWheel).busVoltage(() -> 12.0);
        scheduler.request(peerWheel.drive);
        scheduler.request(guardedWheel.drive);

        scheduler.teleopPeriodic(0.0, 0.02);

        assertEquals(2, calculations.get(), "both controllers are evaluated once");
        assertEquals(0, peer.writes, "a peer registered first must not drive before the later guard is validated");
        assertEquals(0, guarded.writes);
        assertEquals(0.0, peer.volts);
        assertEquals(0.0, guarded.volts);
        assertEquals(1, peerLoop.notifications);
        assertEquals(1, guardedLoop.notifications);
        assertTrue(Double.isNaN(peerLoop.appliedVolts), "anti-windup observes the final neutral output");
        assertTrue(scheduler.powerSnapshot().motors().stream().allMatch(motor -> motor.capAmps() == 0.0));
    }

    private static double current(Motor motor, double volts) {
        MotorConstants constants = MotorConstants.forKind(motor.device.kind()).orElseThrow();
        double emf = motor.speed * 2.0 * Math.PI / constants.raw().speedConstantRadPerSecPerVolt();
        return (volts - emf) / constants.resistance().in(Units.Ohms);
    }
}
