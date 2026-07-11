package ca.frc6390.athena.vendor.rev;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.api.hardware.MotorControllerKinds;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopConfig;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.HardwareBus;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import org.junit.jupiter.api.Test;

class RevBackendTest {
    @Test
    void motorBackendSupportsBuiltInAndEquivalentKeys() {
        RevMotorBackend backend = new RevMotorBackend();

        assertTrue(backend.supports(MotorKinds.NEO));
        assertTrue(backend.supports(MotorKinds.NEO.controlledBy(MotorControllerKinds.SPARK_FLEX)));
        assertTrue(backend.supports((MotorKind) () -> "rev:spark-flex-brushed"));
        assertFalse(backend.supports((MotorKind) () -> "ctre:kraken-x60"));
    }

    @Test
    void encoderBackendSupportsThroughBoreOnly() {
        RevEncoderBackend backend = new RevEncoderBackend();

        assertTrue(backend.supports(EncoderKinds.REV_THROUGH_BORE));
        assertFalse(backend.supports((EncoderKind) () -> "ctre:cancoder"));
        assertTrue(backend.supports(HardwareBus.rio()
                .dio(0).encoder(EncoderKinds.REV_THROUGH_BORE)));
        assertFalse(backend.supports(EncoderDevice.of(EncoderKinds.REV_THROUGH_BORE, 0)));
    }

    @Test
    void motorOptionsSanitizeInvalidValues() {
        RevMotorOptions options = new RevMotorOptions()
                .smartCurrentLimit(-1)
                .openLoopRampSeconds(Double.NaN)
                .closedLoopRampSeconds(1.25);

        assertEquals(0, options.smartCurrentLimitAmps());
        assertEquals(0.0, options.openLoopRampSeconds());
        assertEquals(1.25, options.closedLoopRampSeconds());
    }

    @Test
    void throughBoreReadsAbsolutePositionAndRejectsVelocityWhenUnavailable() {
        RevThroughBoreEncoderHandle handle = new RevThroughBoreEncoderHandle(
                HardwareBus.rio().dio(4).encoder(EncoderKinds.REV_THROUGH_BORE),
                new RevThroughBoreEncoderHandle.ThroughBoreController() {
                    @Override
                    public double absolutePositionRotations() {
                        return 0.42;
                    }

                    @Override
                    public double velocityRotationsPerSecond() {
                        throw new UnsupportedOperationException("velocity unavailable");
                    }
                });

        assertEquals(0.42, handle.absolutePositionRotations(), 1.0e-9);
        assertEquals(0.42, handle.positionRotations(), 1.0e-9);
        assertThrows(UnsupportedOperationException.class, handle::velocityRotationsPerSecond);
    }

    @Test
    void motorConfigurationRunsDuringActivationOnlyOnce() {
        RecordingSparkController controller = new RecordingSparkController();
        RevMotorOptions options = new RevMotorOptions().smartCurrentLimit(35);
        RevMotorHandle handle = new RevMotorHandle(
                MotorDevice.of(MotorKinds.NEO, 1).neutralMode(MotorNeutralMode.BRAKE),
                options,
                controller);

        assertEquals(0, controller.configureCalls);

        handle.activate();
        handle.activate();

        assertEquals(1, controller.configureCalls);
        assertEquals(options, controller.options);
        assertEquals(MotorNeutralMode.BRAKE, controller.device.neutralMode());
    }

    @Test
    void motorFollowerUsesLeaderCanIdAndRequestedDirection() {
        RecordingSparkController leaderController = new RecordingSparkController();
        RecordingSparkController followerController = new RecordingSparkController();
        RevMotorHandle leader = new RevMotorHandle(
                MotorDevice.of(MotorKinds.NEO, 1), null, leaderController);
        RevMotorHandle follower = new RevMotorHandle(
                MotorDevice.of(MotorKinds.NEO, 2), null, followerController);

        follower.follow(leader, true);

        assertEquals(1, followerController.followLeaderId);
        assertTrue(followerController.followInverted);
    }

    @Test
    void motorSensorReadsAreSnapshottedUntilRefresh() {
        RecordingSparkController controller = new RecordingSparkController();
        RevMotorHandle handle = new RevMotorHandle(
                MotorDevice.of(MotorKinds.NEO, 2),
                new RevMotorOptions(),
                controller);

        controller.position = 1.0;
        controller.velocity = 2.0;
        controller.absolutePosition = 0.25;
        controller.absoluteVelocity = 0.5;

        assertEquals(1.0, handle.integratedPositionRotations(), 1.0e-9);
        assertEquals(2.0, handle.integratedVelocityRotationsPerSecond(), 1.0e-9);
        assertEquals(0.25, handle.absolutePositionRotations(), 1.0e-9);
        assertEquals(0.5, handle.absoluteVelocityRotationsPerSecond(), 1.0e-9);
        assertEquals(1, controller.positionReads);
        assertEquals(1, controller.velocityReads);
        assertEquals(1, controller.absolutePositionReads);
        assertEquals(1, controller.absoluteVelocityReads);

        controller.position = 3.0;
        controller.velocity = 4.0;
        controller.absolutePosition = 0.75;
        controller.absoluteVelocity = 1.5;

        assertEquals(1.0, handle.integratedPositionRotations(), 1.0e-9);
        assertEquals(2.0, handle.integratedVelocityRotationsPerSecond(), 1.0e-9);
        assertEquals(0.25, handle.absolutePositionRotations(), 1.0e-9);
        assertEquals(0.5, handle.absoluteVelocityRotationsPerSecond(), 1.0e-9);
        assertEquals(1, controller.positionReads);
        assertEquals(1, controller.velocityReads);
        assertEquals(1, controller.absolutePositionReads);
        assertEquals(1, controller.absoluteVelocityReads);

        handle.refreshInputs();

        assertEquals(3.0, handle.integratedPositionRotations(), 1.0e-9);
        assertEquals(4.0, handle.integratedVelocityRotationsPerSecond(), 1.0e-9);
        assertEquals(0.75, handle.absolutePositionRotations(), 1.0e-9);
        assertEquals(1.5, handle.absoluteVelocityRotationsPerSecond(), 1.0e-9);
        assertEquals(2, controller.positionReads);
        assertEquals(2, controller.velocityReads);
        assertEquals(2, controller.absolutePositionReads);
        assertEquals(2, controller.absoluteVelocityReads);

        handle.setIntegratedPositionRotations(5.0);

        assertEquals(5.0, controller.position, 1.0e-9);
        assertEquals(5.0, handle.integratedPositionRotations(), 1.0e-9);
    }

    @Test
    void motorClosedLoopConfigIsAppliedOnlyWhenChanged() {
        RecordingSparkController controller = new RecordingSparkController();
        RevMotorHandle handle = new RevMotorHandle(
                MotorDevice.of(MotorKinds.NEO, 2),
                new RevMotorOptions(),
                controller);
        MotorClosedLoopConfig config = new MotorClosedLoopConfig(
                1, 0.2, 0.0, 0.01, 0.0, 0.0, 0.1, 0.2, 0.3, null);
        MotorClosedLoopRequest request = MotorClosedLoopRequest.hybrid(config, 1.5);

        handle.setVelocityTargetRotationsPerSecond(2.0, request);
        handle.setVelocityTargetRotationsPerSecond(3.0, request);

        assertEquals(1, controller.configureClosedLoopCalls);
        assertEquals(config, controller.closedLoopConfig);
        assertEquals(3.0, controller.velocityTarget, 1.0e-9);
        assertEquals(1, controller.slot);
        assertEquals(1.5, controller.arbitraryFeedforwardVolts, 1.0e-9);
    }

    private static final class RecordingSparkController implements RevMotorHandle.SparkController {
        private int configureCalls;
        private MotorDevice device;
        private RevMotorOptions options;
        private int positionReads;
        private int velocityReads;
        private int absolutePositionReads;
        private int absoluteVelocityReads;
        private int configureClosedLoopCalls;
        private double position;
        private double velocity;
        private double absolutePosition;
        private double absoluteVelocity;
        private double velocityTarget;
        private double arbitraryFeedforwardVolts;
        private int slot;
        private MotorClosedLoopConfig closedLoopConfig;
        private int followLeaderId = -1;
        private boolean followInverted;

        @Override
        public void configure(MotorDevice device, RevMotorOptions options) {
            configureCalls++;
            this.device = device;
            this.options = options;
        }

        @Override
        public void setPercent(double percent) {}

        @Override
        public void setVoltage(double volts) {}

        @Override
        public void setPositionTarget(double rotations) {}

        @Override
        public void setVelocityTarget(double rotationsPerSecond) {}

        @Override
        public void setSensorPosition(double rotations) {
            position = rotations;
        }

        @Override
        public void setVelocityTarget(double rotationsPerSecond, int slot, double arbitraryFeedforwardVolts) {
            velocityTarget = rotationsPerSecond;
            this.slot = slot;
            this.arbitraryFeedforwardVolts = arbitraryFeedforwardVolts;
        }

        @Override
        public void configureClosedLoop(
                MotorDevice device,
                RevMotorOptions options,
                MotorClosedLoopConfig closedLoopConfig) {
            configureClosedLoopCalls++;
            this.closedLoopConfig = closedLoopConfig;
        }

        @Override
        public void follow(int leaderId, boolean inverted) {
            followLeaderId = leaderId;
            followInverted = inverted;
        }

        @Override
        public void stop() {}

        @Override
        public double positionRotations() {
            positionReads++;
            return position;
        }

        @Override
        public double velocityRotationsPerSecond() {
            velocityReads++;
            return velocity;
        }

        @Override
        public double absolutePositionRotations() {
            absolutePositionReads++;
            return absolutePosition;
        }

        @Override
        public double absoluteVelocityRotationsPerSecond() {
            absoluteVelocityReads++;
            return absoluteVelocity;
        }
    }
}
