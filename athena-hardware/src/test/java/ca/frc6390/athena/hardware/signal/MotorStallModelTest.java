package ca.frc6390.athena.hardware.signal;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.RuntimeScope;
import org.junit.jupiter.api.Test;

class MotorStallModelTest {
    @Test
    void knownMotorDefaultsToModelBasedDetectionWithoutAFixedVoltageThreshold() throws Exception {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1).statorCurrentLimit(120);
        FakeHandle handle = new FakeHandle(motor);
        try (AutoCloseable ignored = motor.bindRuntime(new RuntimeScope("model-stall"), handle)) {
            assertTrue(ignored != null);
            MotorStallSignal stall = motor.stall();
            assertTrue(stall.modelBased());
            assertEquals(MotorStallSignal.MODEL_DURATION_SECONDS, stall.durationSeconds(), 1.0e-12);

            // A loaded motor drawing current at 3 V that is not moving counts as a stall.
            handle.appliedVoltage = 3.0;
            handle.statorCurrent = 40.0;
            handle.velocity = 0.0;
            assertTrue(stall.instantaneousActive());
        }
    }

    @Test
    void reverseDriveStallsAndRunsLikeForward() throws Exception {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1).statorCurrentLimit(40);
        FakeHandle handle = new FakeHandle(motor);
        try (AutoCloseable ignored = motor.bindRuntime(new RuntimeScope("model-stall-reverse"), handle)) {
            assertTrue(ignored != null);
            MotorStallSignal stall = motor.stall();
            // Homing drives in reverse: a loaded, stopped motor at -3.6 V is a stall...
            handle.appliedVoltage = -3.6;
            handle.statorCurrent = -40.0;
            handle.velocity = 0.0;
            assertTrue(stall.instantaneousActive());
            // ...and the same motor spinning freely backwards is not.
            handle.statorCurrent = -2.0;
            handle.velocity = -25.0;
            assertFalse(stall.instantaneousActive());
        }
    }

    @Test
    void freeRunningMotorIsNotAStall() throws Exception {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 2).statorCurrentLimit(120);
        FakeHandle handle = new FakeHandle(motor);
        try (AutoCloseable ignored = motor.bindRuntime(new RuntimeScope("model-free"), handle)) {
            assertTrue(ignored != null);
            MotorStallSignal stall = motor.stall();
            handle.appliedVoltage = 10.0;
            handle.statorCurrent = 2.5;     // near free-run current, below the loaded threshold
            handle.velocity = 0.0;
            assertFalse(stall.instantaneousActive());

            // Moving at the speed the voltage predicts: not a stall even with some load.
            handle.statorCurrent = 30.0;
            double expectedRps = stall.constants().expectedUnloadedSpeedRadPerSec(10.0) / (2.0 * Math.PI);
            handle.velocity = 0.9 * expectedRps;
            assertFalse(stall.instantaneousActive());
        }
    }

    @Test
    void zeroAppliedVoltageIsNeverAStall() throws Exception {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 3).statorCurrentLimit(120);
        FakeHandle handle = new FakeHandle(motor);
        try (AutoCloseable ignored = motor.bindRuntime(new RuntimeScope("model-zero"), handle)) {
            assertTrue(ignored != null);
            handle.appliedVoltage = 0.0;
            handle.statorCurrent = 60.0;
            handle.velocity = 0.0;
            assertFalse(motor.stall().instantaneousActive());
        }
    }

    @Test
    void existingOverridesSwitchToTheFixedThresholdDetector() throws Exception {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 4).statorCurrentLimit(40);
        FakeHandle handle = new FakeHandle(motor);
        try (AutoCloseable ignored = motor.bindRuntime(new RuntimeScope("model-override"), handle)) {
            assertTrue(ignored != null);
            assertFalse(motor.stall().atCurrentLimit(0.9).modelBased());
            assertFalse(motor.stall().velocityBelow(1.0).modelBased());
            assertFalse(motor.stall().commandedVoltageAbove(1.0).modelBased());
            assertTrue(motor.stall().forSeconds(0.3).modelBased());
        }
    }

    @Test
    void unknownMotorKeepsTheLegacyDetector() {
        MotorDevice motor = MotorDevice.of(MotorKinds.MINION, 5).statorCurrentLimit(40);
        assertFalse(motor.stall().modelBased());
    }

    private static final class FakeHandle implements MotorHandle {
        private final MotorDevice device;
        private double appliedVoltage;
        private double statorCurrent;
        private double velocity;

        private FakeHandle(MotorDevice device) {
            this.device = device;
        }

        @Override public MotorDevice device() { return device; }
        @Override public double appliedVoltage() { return appliedVoltage; }
        @Override public double supplyCurrentAmps() { return statorCurrent; }
        @Override public double statorCurrentAmps() { return statorCurrent; }
        @Override public double integratedVelocityRotationsPerSecond() { return velocity; }
    }
}
