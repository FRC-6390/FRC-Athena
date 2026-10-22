package ca.frc6390.athena.robot;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.HardwareGraph;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;
import edu.wpi.first.units.Units;
import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.Executors;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;

class LateHardwareBindingTest {
    @Test
    void directTeleopEntryQueuesLatePanicLeaderAndDependentFollowerWithoutBlockingOrFalseCompletion() throws Exception {
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        Map<Integer, MemoryMotor> handles = new ConcurrentHashMap<>();
        var backend = new MotorBackend() {
            @Override public boolean supports(MotorKind kind) { return true; }
            @Override public MotorHandle create(MotorDevice device) {
                if (device.id() == 97) {
                    entered.countDown();
                    try { release.await(); }
                    catch (InterruptedException exception) { Thread.currentThread().interrupt(); throw new IllegalStateException(exception); }
                }
                MemoryMotor handle = new MemoryMotor(device);
                handles.put(device.id(), handle);
                return handle;
            }
        };
        Healthy healthy = new Healthy();
        Late late = new Late();
        var caller = Executors.newSingleThreadExecutor();
        try (RobotRuntime runtime = RobotRuntime.using(HardwareGraph.using(BackendRegistry.of(backend))).register(healthy)) {
            runtime.request(healthy.drive);
            runtime.teleopPeriodic(0.0, 0.02); // this private entry must switch later binding to background
            var registration = caller.submit(() -> runtime.register(late));
            registration.get(1, TimeUnit.SECONDS);
            assertTrue(entered.await(1, TimeUnit.SECONDS));
            runtime.request(late.waitForZero);
            // Give the follower attempt time to observe its still-pending leader. PANIC must not
            // turn an ordinary dependency wait into a hardware fault or a robot startup exception.
            Thread.sleep(20);
            caller.submit(() -> runtime.teleopPeriodic(0.02, 0.02)).get(1, TimeUnit.SECONDS);
            assertTrue(runtime.hardwareGraph().bindingFailures().isEmpty());
            assertFalse(runtime.hardwareGraph().motor(late.motor).configurationStatus().ready());
            assertFalse(runtime.hardwareGraph().motor(late.follower).configurationStatus().ready());
            assertEquals(0.4, handles.get(96).percent, 1e-9, "healthy mechanisms continue while binding is held");
            assertEquals(0, late.completed, "placeholder zero feedback cannot complete the waiting action");
            release.countDown();
            long deadline = System.nanoTime() + TimeUnit.SECONDS.toNanos(2);
            int cycle = 2;
            while ((!runtime.hardwareGraph().motor(late.motor).configurationStatus().ready()
                    || !runtime.hardwareGraph().motor(late.follower).configurationStatus().ready())
                    && System.nanoTime() < deadline) {
                runtime.teleopPeriodic(cycle++ * 0.02, 0.02);
                Thread.sleep(2);
            }
            assertTrue(runtime.hardwareGraph().motor(late.follower).configurationStatus().ready());
            assertEquals(0, late.completed, "bound feedback remains ten rotations, rather than placeholder zero");
            handles.get(97).position = 0.0;
            runtime.teleopPeriodic(cycle * 0.02, 0.02);
            assertEquals(1, late.completed);
        } finally {
            release.countDown();
            caller.shutdownNow();
        }
    }

    private static final class Healthy implements Mechanism {
        final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 96);
        final Action drive = motor.percent(0.4);
    }

    private static final class Late implements Mechanism {
        int completed;
        final MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 97).failurePolicy(FailurePolicy.PANIC);
        final MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 98).follow(motor).failurePolicy(FailurePolicy.PANIC);
        final Action waitForZero = Actions.sequence()
                .until(() -> motor.position().in(Units.Rotations) == 0.0, motor.percent(0.6))
                .then(Actions.doOnce(() -> completed++));
    }

    private static final class MemoryMotor implements MotorHandle {
        final MotorDevice device;
        volatile double percent;
        volatile double position = 10.0;
        MemoryMotor(MotorDevice device) { this.device = device; }
        @Override public MotorDevice device() { return device; }
        @Override public boolean enforcesSupplyCurrentLimit() { return true; }
        @Override public void setPercentOutput(double value) { percent = value; }
        @Override public double appliedVoltage() { return 12.0 * percent; }
        @Override public double supplyCurrentAmps() { return 0.0; }
        @Override public double statorCurrentAmps() { return 0.0; }
        @Override public double integratedPositionRotations() { return position; }
        @Override public double integratedVelocityRotationsPerSecond() { return 0.0; }
    }
}
