package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.hardware.backend.ConfigurationQueue;
import ca.frc6390.athena.hardware.backend.ConfigurationStatus;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import edu.wpi.first.units.Units;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;

class ImuConfigurationReadinessTest {
    @Test void yawActionWaitsForAcknowledgementBeforeAdvancing() throws Exception { verify(false); }
    @Test void pendingYawPreventsSameCycleRaceCompletion() throws Exception { verify(true); }

    private static void verify(boolean race) throws Exception {
        Heading heading = new Heading(race);
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        try (ConfigurationQueue queue = new ConfigurationQueue("heading", () -> { })) {
            double[] yaw = {12.0};
            ImuHandle handle = new ImuHandle() {
                @Override public ImuDevice device() { return heading.imu; }
                @Override public ConfigurationStatus configurationStatus() { return queue.status(); }
                @Override public double yawDegrees() { return yaw[0]; }
                @Override public void setYawDegrees(double value) {
                    queue.request(ConfigurationQueue.Key.POSITION, value, () -> {
                        entered.countDown();
                        try { release.await(); }
                        catch (InterruptedException exception) { Thread.currentThread().interrupt(); throw new IllegalStateException(exception); }
                        yaw[0] = value;
                    });
                }
            };
            MechanismScheduler scheduler = MechanismScheduler.create(new ActionContext() {
                @Override public ImuHandle imu(ImuDevice device) { return handle; }
            }).register(heading);
            scheduler.request(heading.align);
            scheduler.robotPeriodic(0.0, 0.02);
            assertTrue(entered.await(1, TimeUnit.SECONDS));
            scheduler.robotPeriodic(5.0, 0.02);
            assertFalse(handle.configurationStatus().ready());
            assertEquals(12.0, handle.yawDegrees());
            assertEquals(0, heading.completed);
            release.countDown();
            assertTrue(queue.awaitReady(1000));
            scheduler.robotPeriodic(5.02, 0.02);
            assertEquals(90.0, handle.yawDegrees());
            assertEquals(1, heading.completed);
        } finally { release.countDown(); }
    }

    private static final class Heading implements Mechanism {
        final ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 99);
        int completed;
        final Action align;
        Heading(boolean race) {
            Action set = imu.setYaw(Units.Degrees.of(90.0));
            align = (race ? Actions.race(set, Actions.waitUntil(() -> true)) : set)
                    .then(Actions.waitUntil(() -> imu.yaw().in(Units.Degrees) == 90.0))
                    .then(Actions.doOnce(() -> completed++));
        }
    }
}
