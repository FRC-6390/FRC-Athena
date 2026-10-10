package ca.frc6390.athena.vendor.ctre;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.FocPolicy;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopConfig;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.HardwareBus;
import ca.frc6390.athena.hardware.device.SpiPort;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import org.junit.jupiter.api.Test;

class CtreBackendTest {
    @Test
    void runtimeMotorConfigurationIsAppliedWithoutReplacingTheDeclaration() {
        RecordingTalonController controller = new RecordingTalonController();
        MotorDevice declaration = MotorDevice.of(MotorKinds.KRAKEN_X60, 19).coast();
        CtreMotorHandle handle = new CtreMotorHandle(declaration, new CtreMotorOptions(), controller);

        handle.applyRuntimeConfiguration(new MotorRuntimeConfig(MotorNeutralMode.BRAKE, true, 32, 57));

        assertTrue(handle.supportsRuntimeConfiguration());
        assertEquals(MotorNeutralMode.COAST, handle.device().neutralMode());
        assertTrue(controller.brake);
        assertTrue(controller.inverted);
        assertEquals(32, controller.supplyCurrentLimitAmps);
        assertEquals(57, controller.statorCurrentLimitAmps);
    }

    @Test
    void motorBackendSupportsBuiltInAndEquivalentKeys() {
        CtreMotorBackend backend = new CtreMotorBackend();

        assertTrue(backend.supports(MotorKinds.FALCON_500));
        assertTrue(backend.supports(MotorKinds.MINION));
        assertTrue(backend.supports(() -> "ctre:talon-fxs/minion"));
        assertTrue(backend.supports(() -> "ctre:kraken-x60"));
        assertFalse(backend.supports((MotorKind) () -> "rev:spark-max-brushless"));
    }

    @Test
    void encoderBackendSupportsCanCoderOnly() {
        CtreEncoderBackend backend = new CtreEncoderBackend();

        assertTrue(backend.supports(EncoderKinds.CANCODER));
        assertTrue(backend.supports((EncoderKind) () -> "ctre:cancoder"));
        assertFalse(backend.supports((EncoderKind) () -> "rev:through-bore"));
        assertTrue(backend.supports(EncoderDevice.of(EncoderKinds.CANCODER, 1)));
        assertFalse(backend.supports(HardwareBus.rio().dio(1).encoder(EncoderKinds.CANCODER)));
    }

    @Test
    void imuBackendSupportsPigeon2Only() {
        CtreImuBackend backend = new CtreImuBackend();

        assertTrue(backend.supports(ImuKinds.PIGEON_2));
        assertTrue(backend.supports((ImuKind) () -> "ctre:pigeon-2"));
        assertFalse(backend.supports((ImuKind) () -> "studica:navx"));
        assertTrue(backend.supports(ImuDevice.of(ImuKinds.PIGEON_2, 1)));
        assertFalse(backend.supports(HardwareBus.rio().spi(SpiPort.MXP).imu(ImuKinds.PIGEON_2)));
    }

    @Test
    void motorOptionsClampNegativeCurrentLimits() {
        CtreMotorOptions options = new CtreMotorOptions()
                .supplyCurrentLimit(-1)
                .statorCurrentLimit(-2)
                .torqueCurrentLimit(35);

        assertEquals(0, options.supplyCurrentLimitAmps());
        assertEquals(0, options.statorCurrentLimitAmps());
        assertEquals(35, options.torqueCurrentLimitAmps());
    }

    @Test
    void motorConfigurationRunsDuringActivationOnlyOnce() {
        RecordingTalonController controller = new RecordingTalonController();
        CtreMotorHandle handle = new CtreMotorHandle(
                MotorDevice.of(MotorKinds.FALCON_500, 1).neutralMode(MotorNeutralMode.BRAKE).inverted(),
                new CtreMotorOptions(),
                controller);

        assertEquals(0, controller.outputConfigurationCalls);

        handle.activate();
        handle.activate();

        assertEquals(1, controller.outputConfigurationCalls);
        assertTrue(controller.brake);
        assertTrue(controller.inverted);
    }

    @Test
    void genericCurrentLimitConfiguresCtreSupplyLimit() {
        RecordingTalonController controller = new RecordingTalonController();
        CtreMotorHandle handle = new CtreMotorHandle(
                MotorDevice.of(MotorKinds.KRAKEN_X60, 1).currentLimit(37),
                new CtreMotorOptions(),
                controller);

        handle.activate();

        assertEquals(1, controller.currentLimitConfigurationCalls);
        assertEquals(37, controller.supplyCurrentLimitAmps);
        assertEquals(0, controller.statorCurrentLimitAmps);
        assertEquals(0, controller.torqueCurrentLimitAmps);
    }

    @Test
    void directSupplyAndStatorLimitsConfigureCtreController() {
        RecordingTalonController controller = new RecordingTalonController();
        CtreMotorHandle handle = new CtreMotorHandle(
                MotorDevice.of(MotorKinds.KRAKEN_X60, 1)
                        .currentLimit(37)
                        .supplyCurrentLimit(45)
                        .statorCurrentLimit(80),
                new CtreMotorOptions(),
                controller);

        handle.activate();

        assertEquals(45, controller.supplyCurrentLimitAmps);
        assertEquals(80, controller.statorCurrentLimitAmps);
        assertEquals(0, controller.torqueCurrentLimitAmps);
    }

    @Test
    void ctreCurrentLimitOptionsOverrideAndExtendGenericLimit() {
        RecordingTalonController controller = new RecordingTalonController();
        CtreMotorHandle handle = new CtreMotorHandle(
                MotorDevice.of(MotorKinds.KRAKEN_X60, 1)
                        .currentLimit(37)
                        .supplyCurrentLimit(42)
                        .statorCurrentLimit(70),
                new CtreMotorOptions()
                        .supplyCurrentLimit(45)
                        .statorCurrentLimit(80)
                        .torqueCurrentLimit(60),
                controller);

        handle.activate();

        assertEquals(45, controller.supplyCurrentLimitAmps);
        assertEquals(80, controller.statorCurrentLimitAmps);
        assertEquals(60, controller.torqueCurrentLimitAmps);
    }

    @Test
    void motorFollowerUsesLeaderCanIdAndRequestedDirection() {
        RecordingTalonController leaderController = new RecordingTalonController();
        RecordingTalonController followerController = new RecordingTalonController();
        CtreMotorHandle leader = new CtreMotorHandle(
                MotorDevice.of(MotorKinds.KRAKEN_X60, 1), null, leaderController);
        CtreMotorHandle follower = new CtreMotorHandle(
                MotorDevice.of(MotorKinds.KRAKEN_X60, 2), null, followerController);

        follower.follow(leader, true);

        assertEquals(1, followerController.followLeaderId);
        assertTrue(followerController.followInverted);
    }

    @Test
    void motorFollowerOnDifferentCanBusFailsLoudly() {
        RecordingTalonController followerController = new RecordingTalonController();
        CtreMotorHandle leader = new CtreMotorHandle(
                MotorDevice.of(MotorKinds.KRAKEN_X60, 1).canbus("canivore"), null, new RecordingTalonController());
        CtreMotorHandle follower = new CtreMotorHandle(
                MotorDevice.of(MotorKinds.KRAKEN_X60, 2), null, followerController);

        IllegalStateException failure = assertThrows(IllegalStateException.class,
                () -> follower.follow(leader, false));

        assertTrue(failure.getMessage().contains("canivore"));
        assertTrue(failure.getMessage().contains("rio"));
        assertEquals(-1, followerController.followLeaderId);
    }

    @Test
    void encoderActivationOnlyWarnsAboutStoredConfigByDefault() {
        RecordingCancoderController controller = new RecordingCancoderController();
        controller.stored = new CtreEncoderHandle.MagnetConfig(0.125, false, 1.0);
        java.util.List<String> warnings = new java.util.ArrayList<>();
        CtreEncoderHandle handle = new CtreEncoderHandle(
                EncoderDevice.of(EncoderKinds.CANCODER, 3), controller, warnings::add);

        handle.activate();
        handle.activate();

        assertEquals(0, controller.baselineCalls);
        assertEquals(1, warnings.size());
        assertTrue(warnings.get(0).contains("0.125"));
        assertTrue(warnings.get(0).contains("CW+"));
        assertTrue(warnings.get(0).contains(EncoderDevice.of(EncoderKinds.CANCODER, 3).defaultName()));

        RecordingCancoderController matching = new RecordingCancoderController();
        java.util.List<String> quiet = new java.util.ArrayList<>();
        new CtreEncoderHandle(EncoderDevice.of(EncoderKinds.CANCODER, 4), matching, quiet::add).activate();
        assertTrue(quiet.isEmpty());
    }

    @Test
    void encoderBaselineIsWrittenOnlyWhenOptedInAndReportsFailures() {
        RecordingCancoderController controller = new RecordingCancoderController();
        java.util.List<String> warnings = new java.util.ArrayList<>();
        CtreEncoderHandle handle = new CtreEncoderHandle(
                EncoderDevice.of(EncoderKinds.CANCODER, 3).applyBaselineConfig().offset(0.2).inverted(),
                controller,
                warnings::add);

        handle.activate();
        handle.activate();
        assertEquals(1, controller.baselineCalls);
        assertTrue(warnings.isEmpty());

        RecordingCancoderController failing = new RecordingCancoderController();
        failing.baselineFailure = "CAN frame not received";
        CtreEncoderHandle failingHandle = new CtreEncoderHandle(
                EncoderDevice.of(EncoderKinds.CANCODER, 4).applyBaselineConfig(), failing);
        IllegalStateException failure = assertThrows(IllegalStateException.class, failingHandle::activate);
        assertTrue(failure.getMessage().contains("CAN frame not received"));
    }

    @Test
    void pendingPositionSetBridgesFramesReceivedBeforeTheSet() {
        PendingPositionSet pending = new PendingPositionSet();
        assertEquals(1.0, pending.resolve(1.0, 5.0), 1.0e-9);

        pending.set(4.0, 10.0);

        assertEquals(4.0, pending.resolve(1.0, 9.99), 1.0e-9);
        assertEquals(4.0, pending.resolve(1.0, 10.0), 1.0e-9);
        assertEquals(4.01, pending.resolve(4.01, 10.02), 1.0e-9);
        assertEquals(4.5, pending.resolve(4.5, 9.0), 1.0e-9);
    }

    @Test
    void motorInputsAreCachedUntilNextRefresh() {
        RecordingTalonController controller = new RecordingTalonController();
        CtreMotorHandle handle = new CtreMotorHandle(MotorDevice.of(MotorKinds.FALCON_500, 2), null, controller);

        handle.refreshInputs();

        assertEquals(10.0, handle.integratedPositionRotations(), 1.0e-9);
        assertEquals(5.0, handle.integratedVelocityRotationsPerSecond(), 1.0e-9);
        assertEquals(10.0, handle.integratedPositionRotations(), 1.0e-9);
        assertEquals(1, controller.positionCalls);
        assertEquals(1, controller.velocityCalls);

        handle.setIntegratedPositionRotations(6.0);

        assertEquals(6.0, handle.integratedPositionRotations(), 1.0e-9);
        assertEquals(6.0, controller.sensorPosition, 1.0e-9);
    }

    @Test
    void motorClosedLoopConfigIsAppliedOnlyWhenChanged() {
        RecordingTalonController controller = new RecordingTalonController();
        CtreMotorHandle handle = new CtreMotorHandle(MotorDevice.of(MotorKinds.FALCON_500, 2), null, controller);
        MotorClosedLoopConfig config = new MotorClosedLoopConfig(
                0, 0.2, 0.0, 0.01, 0.0, 0.1, 0.2, 0.3, 0.4, FocPolicy.DISABLED);
        MotorClosedLoopRequest request = MotorClosedLoopRequest.hybrid(config, 1.5);

        handle.setPositionTargetRotations(2.0, request);
        handle.setPositionTargetRotations(3.0, request);

        assertEquals(1, controller.configureSlotCalls);
        assertEquals(config, controller.config);
        assertEquals(3.0, controller.positionTarget, 1.0e-9);
        assertEquals(1.5, controller.feedforwardVolts, 1.0e-9);
    }

    @Test
    void failedSlotConfigurationIsNotCachedAndIsRetried() {
        RecordingTalonController controller = new RecordingTalonController();
        controller.configurationSucceeds = false;
        CtreMotorHandle handle = new CtreMotorHandle(MotorDevice.of(MotorKinds.FALCON_500, 2), null, controller);
        MotorClosedLoopRequest request = MotorClosedLoopRequest.device(new MotorClosedLoopConfig(
                0, 0.2, 0.01, 0.03, 0.0, 0.1, 0.2, 0.3, 0.4, FocPolicy.DISABLED));

        assertThrows(IllegalStateException.class,
                () -> handle.setPositionTargetRotations(2.0, request));
        assertEquals(1, controller.configureSlotCalls);

        controller.configurationSucceeds = true;
        handle.setPositionTargetRotations(2.0, request);
        handle.setPositionTargetRotations(3.0, request);

        assertEquals(2, controller.configureSlotCalls);
        assertEquals(3.0, controller.positionTarget, 1.0e-9);
    }

    @Test
    void optionalFocFallsBackAfterLicenseFault() {
        RecordingTalonController controller = new RecordingTalonController();
        controller.licenseFault = true;
        CtreMotorHandle handle = new CtreMotorHandle(
                MotorDevice.of(MotorKinds.FALCON_500, 2),
                new CtreMotorOptions().foc(FocPolicy.ENABLE_IF_AVAILABLE),
                controller);
        MotorClosedLoopRequest request = MotorClosedLoopRequest.device(MotorClosedLoopConfig.empty());

        handle.setVelocityTargetRotationsPerSecond(4.0, request);

        assertEquals(2, controller.velocityTargetCalls);
        assertFalse(controller.enableFoc);
    }

    @Test
    void encoderInputsAreCachedUntilNextRefresh() {
        RecordingCancoderController controller = new RecordingCancoderController();
        CtreEncoderHandle handle = new CtreEncoderHandle(EncoderDevice.of(EncoderKinds.CANCODER, 3), controller);

        handle.refreshInputs();

        assertEquals(1.5, handle.positionRotations(), 1.0e-9);
        assertEquals(0.25, handle.absolutePositionRotations(), 1.0e-9);
        assertEquals(2.5, handle.velocityRotationsPerSecond(), 1.0e-9);
        assertEquals(1.5, handle.positionRotations(), 1.0e-9);
        assertEquals(1, controller.positionCalls);
        assertEquals(1, controller.absolutePositionCalls);
        assertEquals(1, controller.velocityCalls);

        handle.setPositionRotations(4.5);

        assertEquals(4.5, controller.setPositionRotations, 1.0e-9);
        assertEquals(4.5, handle.positionRotations(), 1.0e-9);
    }

    @Test
    void pigeonInputsAreCachedUntilNextRefresh() {
        RecordingPigeonController controller = new RecordingPigeonController();
        CtrePigeon2Handle handle = new CtrePigeon2Handle(ImuDevice.of(ImuKinds.PIGEON_2, 4), controller);

        handle.refreshInputs();

        assertEquals(12.0, handle.yawDegrees(), 1.0e-9);
        assertEquals(3.0, handle.pitchDegrees(), 1.0e-9);
        assertEquals(4.0, handle.rollDegrees(), 1.0e-9);
        assertEquals(5.0, handle.yawRateDegreesPerSecond(), 1.0e-9);
        assertEquals(6.0, handle.pitchRateDegreesPerSecond(), 1.0e-9);
        assertEquals(7.0, handle.rollRateDegreesPerSecond(), 1.0e-9);
        assertEquals(0.1, handle.linearAccelerationXG(), 1.0e-9);
        assertEquals(0.2, handle.linearAccelerationYG(), 1.0e-9);
        assertEquals(0.3, handle.linearAccelerationZG(), 1.0e-9);
        assertTrue(handle.isConnected());
        assertFalse(handle.isCalibrating());
        assertTrue(Double.isFinite(handle.lastUpdateSeconds()));
        assertEquals(12.0, handle.yawDegrees(), 1.0e-9);
        assertEquals(1, controller.yawCalls);
        assertEquals(1, controller.pitchCalls);
        assertEquals(1, controller.rollCalls);
    }

    private static final class RecordingTalonController implements CtreMotorHandle.TalonController {
        private int outputConfigurationCalls;
        private boolean brake;
        private int configureSlotCalls;
        private int velocityTargetCalls;
        private boolean licenseFault;
        private boolean configurationSucceeds = true;
        private boolean enableFoc;
        private double positionTarget;
        private double feedforwardVolts;
        private MotorClosedLoopConfig config;
        private int followLeaderId = -1;
        private boolean followInverted;
        private boolean inverted;
        private double sensorPosition = 10.0;
        private int currentLimitConfigurationCalls;
        private int supplyCurrentLimitAmps;
        private int statorCurrentLimitAmps;
        private int torqueCurrentLimitAmps;

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
            sensorPosition = rotations;
        }

        @Override
        public boolean setPositionTarget(double rotations, int slot, double feedforwardVolts, boolean enableFoc) {
            positionTarget = rotations;
            this.feedforwardVolts = feedforwardVolts;
            this.enableFoc = enableFoc;
            return licenseFault && enableFoc;
        }

        @Override
        public boolean setVelocityTarget(
                double rotationsPerSecond,
                int slot,
                double feedforwardVolts,
                boolean enableFoc) {
            velocityTargetCalls++;
            this.feedforwardVolts = feedforwardVolts;
            this.enableFoc = enableFoc;
            return licenseFault && enableFoc;
        }

        @Override
        public boolean configureSlot(MotorClosedLoopConfig config) {
            configureSlotCalls++;
            this.config = config;
            return configurationSucceeds;
        }

        @Override
        public void follow(int leaderId, boolean inverted) {
            followLeaderId = leaderId;
            followInverted = inverted;
        }

        @Override
        public void stop() {}

        @Override
        public boolean configureOutput(boolean brake, boolean inverted) {
            outputConfigurationCalls++;
            this.brake = brake;
            this.inverted = inverted;
            return true;
        }

        @Override
        public boolean configureCurrentLimits(int supplyAmps, int statorAmps, int torqueAmps) {
            currentLimitConfigurationCalls++;
            supplyCurrentLimitAmps = supplyAmps;
            statorCurrentLimitAmps = statorAmps;
            torqueCurrentLimitAmps = torqueAmps;
            return true;
        }

        @Override
        public double positionRotations() {
            positionCalls++;
            return sensorPosition;
        }

        @Override
        public double velocityRotationsPerSecond() {
            velocityCalls++;
            return 5.0;
        }

        private int positionCalls;
        private int velocityCalls;
    }

    private static final class RecordingCancoderController implements CtreEncoderHandle.CANCoderController {
        private int baselineCalls;
        private String baselineFailure;
        private CtreEncoderHandle.MagnetConfig stored = new CtreEncoderHandle.MagnetConfig(0.0, true, 0.5);
        private int positionCalls;
        private int absolutePositionCalls;
        private int velocityCalls;
        private double setPositionRotations;

        @Override
        public double positionRotations() {
            positionCalls++;
            return 1.5;
        }

        @Override
        public double absolutePositionRotations() {
            absolutePositionCalls++;
            return 0.25;
        }

        @Override
        public double velocityRotationsPerSecond() {
            velocityCalls++;
            return 2.5;
        }

        @Override
        public boolean setPositionRotations(double rotations) {
            setPositionRotations = rotations;
            return true;
        }

        @Override
        public String configureBaseline() {
            baselineCalls++;
            return baselineFailure;
        }

        @Override
        public CtreEncoderHandle.MagnetConfig readMagnetConfig() {
            return stored;
        }
    }

    private static final class RecordingPigeonController implements CtrePigeon2Handle.Pigeon2Controller {
        private int yawCalls;
        private int pitchCalls;
        private int rollCalls;

        @Override
        public double yawDegrees() {
            yawCalls++;
            return 12.0;
        }

        @Override
        public double pitchDegrees() {
            pitchCalls++;
            return 3.0;
        }

        @Override
        public double rollDegrees() {
            rollCalls++;
            return 4.0;
        }

        @Override
        public double yawRateDegreesPerSecond() {
            return 5.0;
        }

        @Override public double pitchRateDegreesPerSecond() { return 6.0; }
        @Override public double rollRateDegreesPerSecond() { return 7.0; }

        @Override
        public double linearAccelerationXG() {
            return 0.1;
        }

        @Override
        public double linearAccelerationYG() {
            return 0.2;
        }

        @Override
        public double linearAccelerationZG() {
            return 0.3;
        }

        @Override
        public boolean setYawDegrees(double yawDegrees) {
            return true;
        }

        @Override
        public void reset() {}

        @Override
        public void close() {}
    }
}
