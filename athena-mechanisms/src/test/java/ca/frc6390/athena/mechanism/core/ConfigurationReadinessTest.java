package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.ConfigurationQueue;
import ca.frc6390.athena.hardware.backend.ConfigurationStatus;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.TimeUnit;
import org.junit.jupiter.api.Test;

class ConfigurationReadinessTest {
    @Test
    void pendingZeroDoesNotCompleteItsSequenceOrFreezeAHealthyMechanism() throws Exception {
        assertPendingZero(0);
    }

    @Test
    void pendingZeroFencesARaceCompletionConditionInTheSameCycle() throws Exception {
        assertPendingZero(1);
    }

    @Test
    void pendingZeroFencesTheSequenceStepCompletionPredicateInTheSameCycle() throws Exception {
        assertPendingZero(2);
    }

    @Test
    void pendingZeroFencesLaterParallelBranchesInTheSameCycle() throws Exception {
        assertPendingZero(3);
    }

    private static void assertPendingZero(int mode) throws Exception {
        Axis axis = new Axis(84, mode);
        Axis healthy = new Axis(85);
        CountDownLatch entered = new CountDownLatch(1);
        CountDownLatch release = new CountDownLatch(1);
        try (ConfigurationQueue queue = new ConfigurationQueue("zero", () -> { })) {
            double[] healthyOutput = {0.0};
            MotorHandle pendingMotor = new MotorHandle() {
                @Override public MotorDevice device() { return axis.motor; }
                @Override public ConfigurationStatus configurationStatus() { return queue.status(); }
                @Override public void setPercentOutput(double value) { }
                @Override public double appliedVoltage() { return 0.0; }
                @Override public double supplyCurrentAmps() { return 0.0; }
                @Override public double integratedVelocityRotationsPerSecond() { return 0.0; }
            };
            MotorHandle healthyMotor = new MotorHandle() {
                @Override public MotorDevice device() { return healthy.motor; }
                @Override public boolean enforcesSupplyCurrentLimit() { return true; }
                @Override public void setPercentOutput(double value) { healthyOutput[0] = value; }
                @Override public double appliedVoltage() { return 0.0; }
                @Override public double supplyCurrentAmps() { return 0.0; }
                @Override public double integratedVelocityRotationsPerSecond() { return 0.0; }
            };
            EncoderHandle encoder = new EncoderHandle() {
                @Override public EncoderDevice device() { return axis.encoder; }
                @Override public ConfigurationStatus configurationStatus() { return queue.status(); }
                @Override public double positionRotations() { return 10.0; }
                @Override public void setPositionRotations(double value) {
                    queue.request(ConfigurationQueue.Key.POSITION, value, () -> {
                        entered.countDown();
                        try { release.await(); }
                        catch (InterruptedException exception) { Thread.currentThread().interrupt(); throw new IllegalStateException(exception); }
                    });
                }
            };
            ActionContext hardware = new ActionContext() {
                @Override public MotorHandle motor(MotorDevice device) { return device == axis.motor ? pendingMotor : healthyMotor; }
                @Override public EncoderHandle encoder(EncoderDevice device) {
                    return device == axis.encoder ? encoder : new EncoderHandle() {
                        @Override public EncoderDevice device() { return device; }
                        @Override public double positionRotations() { return 0.0; }
                    };
                }
            };
            MechanismScheduler scheduler = MechanismScheduler.create(hardware).register(axis).register(healthy);
            scheduler.request(axis.zero);
            scheduler.request(healthy.drive);
            scheduler.robotPeriodic(0.0, 0.02);
            assertTrue(entered.await(1, TimeUnit.SECONDS));
            scheduler.robotPeriodic(5.0, 0.02);
            assertFalse(queue.status().ready());
            assertEquals(0, axis.afterZero, "the next action waits for the zero acknowledgement");
            assertEquals(0, axis.siblingRuns, "later branches must not run against an unacknowledged position frame");
            assertEquals(0.4, healthyOutput[0], 1e-9);
            release.countDown();
            assertTrue(queue.awaitReady(1000));
            scheduler.robotPeriodic(5.02, 0.02);
            assertEquals(1, axis.afterZero);
        } finally { release.countDown(); }
    }

    private static final class Axis implements Mechanism {
        final MotorDevice motor;
        final EncoderDevice encoder;
        final Action zero;
        final Action drive;
        int afterZero;
        int siblingRuns;
        Axis(int id) {
            this(id, 0);
        }
        Axis(int id, int mode) {
            motor = MotorDevice.of(MotorKinds.KRAKEN_X60, id);
            encoder = motor.encoder();
            Action set = Actions.setPosition(encoder, 0.0);
            Action body = switch (mode) {
                case 1 -> Actions.race(set, Actions.waitUntil(() -> true));
                case 2 -> Actions.sequence().until(() -> true, set);
                case 3 -> Actions.parallel(set, Actions.doOnce(() -> siblingRuns++));
                default -> set;
            };
            zero = body.then(Actions.doOnce(() -> afterZero++));
            drive = motor.percent(0.4);
        }
    }
}
