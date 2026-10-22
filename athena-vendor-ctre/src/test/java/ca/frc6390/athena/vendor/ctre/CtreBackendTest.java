package ca.frc6390.athena.vendor.ctre;

import static edu.wpi.first.units.Units.Amps;
import edu.wpi.first.units.Units;
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
    private static ca.frc6390.athena.hardware.backend.ConfigurationStatus configuration(Object handle) {
        return handle instanceof ca.frc6390.athena.hardware.backend.MotorHandle motor
                ? motor.configurationStatus()
                : handle instanceof ca.frc6390.athena.hardware.backend.EncoderHandle encoder
                        ? encoder.configurationStatus()
                        : ((ca.frc6390.athena.hardware.backend.ImuHandle) handle).configurationStatus();
    }

    private static void awaitConfiguration(Object handle) {
        awaitCondition(() -> !configuration(handle).pending());
        assertTrue(configuration(handle).ready(), () -> configuration(handle).error());
    }

    private static void awaitFailure(Object handle) {
        awaitCondition(() -> configuration(handle).failed());
    }

    private static void awaitCondition(java.util.function.BooleanSupplier condition) {
        long deadline = System.nanoTime() + java.util.concurrent.TimeUnit.SECONDS.toNanos(3);
        while (!condition.getAsBoolean() && System.nanoTime() < deadline) {
            try { Thread.sleep(1); }
            catch (InterruptedException exception) { Thread.currentThread().interrupt(); throw new AssertionError(exception); }
        }
        assertTrue(condition.getAsBoolean(), "background configuration completed");
    }

    @Test
    void runtimeMotorConfigurationIsAppliedWithoutReplacingTheDeclaration() {
        RecordingTalonController controller = new RecordingTalonController();
        MotorDevice declaration = MotorDevice.of(MotorKinds.KRAKEN_X60, 19).coast();
        CtreMotorHandle handle = new CtreMotorHandle(declaration, new CtreMotorOptions(), controller);

        handle.applyRuntimeConfiguration(new MotorRuntimeConfig(MotorNeutralMode.BRAKE, true, 32, 57));

        awaitConfiguration(handle);

        assertTrue(handle.supportsRuntimeConfiguration());
        assertTrue(handle.enforcesSupplyCurrentLimit());
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
                .supplyCurrentLimit(Amps.of(-1))
                .statorCurrentLimit(Amps.of(-2))
                .torqueCurrentLimit(Amps.of(35));

        assertEquals(0, options.supplyCurrentLimit().in(Amps));
        assertEquals(0, options.statorCurrentLimit().in(Amps));
        assertEquals(35, options.torqueCurrentLimit().in(Amps));
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

        awaitConfiguration(handle);
        handle.activate();
        awaitConfiguration(handle);

        assertEquals(1, controller.outputConfigurationCalls);
        assertTrue(controller.brake);
        assertTrue(controller.inverted);
    }

    @Test
    void genericCurrentLimitConfiguresCtreSupplyLimit() {
        RecordingTalonController controller = new RecordingTalonController();
        CtreMotorHandle handle = new CtreMotorHandle(
                MotorDevice.of(MotorKinds.KRAKEN_X60, 1).currentLimit(Units.Amps.of(37)),
                new CtreMotorOptions(),
                controller);

        handle.activate();

        awaitConfiguration(handle);

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
                        .currentLimit(Units.Amps.of(37))
                        .supplyCurrentLimit(Units.Amps.of(45))
                        .statorCurrentLimit(Units.Amps.of(80)),
                new CtreMotorOptions(),
                controller);

        handle.activate();

        awaitConfiguration(handle);

        assertEquals(45, controller.supplyCurrentLimitAmps);
        assertEquals(80, controller.statorCurrentLimitAmps);
        assertEquals(0, controller.torqueCurrentLimitAmps);
    }

    @Test
    void ctreCurrentLimitOptionsOverrideAndExtendGenericLimit() {
        RecordingTalonController controller = new RecordingTalonController();
        CtreMotorHandle handle = new CtreMotorHandle(
                MotorDevice.of(MotorKinds.KRAKEN_X60, 1)
                        .currentLimit(Units.Amps.of(37))
                        .supplyCurrentLimit(Units.Amps.of(42))
                        .statorCurrentLimit(Units.Amps.of(70)),
                new CtreMotorOptions()
                        .supplyCurrentLimit(Amps.of(45))
                        .statorCurrentLimit(Amps.of(80))
                        .torqueCurrentLimit(Amps.of(60)),
                controller);

        handle.activate();

        awaitConfiguration(handle);

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

        awaitConfiguration(follower);

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

        awaitConfiguration(handle);
        handle.activate();
        awaitConfiguration(handle);

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

        awaitConfiguration(handle);
        handle.activate();
        awaitConfiguration(handle);
        assertEquals(1, controller.baselineCalls);
        assertTrue(warnings.isEmpty());

        RecordingCancoderController failing = new RecordingCancoderController();
        failing.baselineFailure = "CAN frame not received";
        CtreEncoderHandle failingHandle = new CtreEncoderHandle(
                EncoderDevice.of(EncoderKinds.CANCODER, 4).applyBaselineConfig(), failing);
        failingHandle.activate();
        awaitFailure(failingHandle);
        assertTrue(failingHandle.configurationStatus().error().contains("CAN frame not received"));
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

        awaitConfiguration(handle);

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

        awaitConfiguration(handle);

        handle.setPositionTargetRotations(2.0, request);
        handle.setPositionTargetRotations(3.0, request);
        awaitConfiguration(handle);
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

        handle.setPositionTargetRotations(2.0, request);
        awaitFailure(handle);
        assertEquals(1, controller.configureSlotCalls);

        controller.configurationSucceeds = true;
        try { Thread.sleep(110); }
        catch (InterruptedException exception) { Thread.currentThread().interrupt(); throw new AssertionError(exception); }
        handle.setPositionTargetRotations(2.0, request);
        awaitConfiguration(handle);
        handle.setPositionTargetRotations(2.0, request);
        handle.setPositionTargetRotations(3.0, request);
        awaitConfiguration(handle);
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

        awaitConfiguration(handle);

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

        awaitConfiguration(handle);

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

    @Test
    void faultedReportsTheFatalFaultCapturedByTheLastRefresh() {
        RecordingTalonController controller = new RecordingTalonController();
        CtreMotorHandle handle = new CtreMotorHandle(
                MotorDevice.of(MotorKinds.KRAKEN_X60, 5), new CtreMotorOptions(), controller);

        handle.refreshInputs();
        assertFalse(handle.faulted());

        controller.fatalFault = true;
        assertFalse(handle.faulted(), "faulted() reads the cached refresh, not the controller");
        assertEquals(1, controller.fatalFaultReads);

        handle.refreshInputs();
        assertTrue(handle.faulted());

        controller.fatalFault = false;
        handle.refreshInputs();
        assertFalse(handle.faulted());
        assertEquals(3, controller.fatalFaultReads);
    }

    @Test
    void blockedSlotAndRuntimeConfigurationAreOrderedAndGateOnlyTheirOwnMotor() throws Exception {
        var entered = new java.util.concurrent.CountDownLatch(1);
        var release = new java.util.concurrent.CountDownLatch(1);
        java.util.List<String> writes = new java.util.ArrayList<>();
        RecordingTalonController controller = new RecordingTalonController() {
            @Override public boolean configureSlot(MotorClosedLoopConfig config) {
                if (config.p() == 0.1) {
                    entered.countDown();
                    try { release.await(); }
                    catch (InterruptedException exception) { Thread.currentThread().interrupt(); throw new IllegalStateException(exception); }
                }
                writes.add("slot:" + config.p());
                return super.configureSlot(config);
            }
            @Override public boolean configureOutput(boolean brake, boolean inverted) {
                writes.add("runtime");
                return super.configureOutput(brake, inverted);
            }
        };
        RecordingTalonController healthyController = new RecordingTalonController();
        try (CtreMotorHandle handle = new CtreMotorHandle(MotorDevice.of(MotorKinds.KRAKEN_X60, 90), null, controller);
                CtreMotorHandle healthy = new CtreMotorHandle(MotorDevice.of(MotorKinds.KRAKEN_X60, 91), null, healthyController)) {
            MotorClosedLoopRequest first = MotorClosedLoopRequest.device(new MotorClosedLoopConfig(
                    0, 0.1, 0, 0, 0, 0, 0, 0, 0, FocPolicy.DISABLED));
            MotorClosedLoopRequest latest = MotorClosedLoopRequest.device(new MotorClosedLoopConfig(
                    0, 0.3, 0, 0, 0, 0, 0, 0, 0, FocPolicy.DISABLED));
            handle.setPositionTargetRotations(1.0, first);
            assertTrue(entered.await(1, java.util.concurrent.TimeUnit.SECONDS));
            handle.applyRuntimeConfiguration(new MotorRuntimeConfig(MotorNeutralMode.BRAKE, true, 20, 40));
            handle.setPositionTargetRotations(2.0, latest);
            handle.setVoltage(8.0);
            assertTrue(Double.isNaN(controller.lastVolts));
            healthy.setVoltage(6.0);
            assertEquals(6.0, healthyController.lastVolts);
            assertFalse(handle.configurationStatus().ready());
            release.countDown();
            awaitConfiguration(handle);
            assertEquals(java.util.List.of("slot:0.1", "runtime", "slot:0.3"), writes);
            handle.setPositionTargetRotations(2.0, latest);
            assertEquals(2.0, controller.positionTarget);
        } finally { release.countDown(); }
    }

    @Test
    void leavingFollowerModeSupersedesAnInFlightOldFollowBeforeAcknowledgingPromotion() throws Exception {
        var entered = new java.util.concurrent.CountDownLatch(1);
        var release = new java.util.concurrent.CountDownLatch(1);
        var currentLeader = new java.util.concurrent.atomic.AtomicInteger(-1);
        RecordingTalonController controller = new RecordingTalonController() {
            @Override public void follow(int id, boolean inverted) {
                entered.countDown();
                try { release.await(); }
                catch (InterruptedException exception) { Thread.currentThread().interrupt(); throw new IllegalStateException(exception); }
                currentLeader.set(id);
            }
            @Override public void stop() { currentLeader.set(-1); }
        };
        try (CtreMotorHandle leader = new CtreMotorHandle(MotorDevice.of(MotorKinds.KRAKEN_X60, 94), null,
                    new RecordingTalonController());
                CtreMotorHandle follower = new CtreMotorHandle(MotorDevice.of(MotorKinds.KRAKEN_X60, 95), null, controller)) {
            follower.follow(leader, false);
            assertTrue(entered.await(1, java.util.concurrent.TimeUnit.SECONDS));
            follower.stopFollowing();
            assertFalse(follower.configurationStatus().ready());
            release.countDown();
            awaitConfiguration(follower);
            assertEquals(-1, currentLeader.get(), "the old follow cannot remain active or retry after promotion");
        } finally { release.countDown(); }
    }

    private static class RecordingTalonController implements CtreMotorHandle.TalonController {
        private double lastVolts = Double.NaN;
        private boolean fatalFault;
        private int fatalFaultReads;
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
        public boolean fatalFault() {
            fatalFaultReads++;
            return fatalFault;
        }

        @Override
        public void setPercent(double percent) {}

        @Override
        public void setVoltage(double volts) { lastVolts = volts; }

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

    @Test
    void heldYawMutationKeepsOldSnapshotUnusableAndOrdersLatestResetWithoutBlockingHealthyReads() throws Exception {
        var entered = new java.util.concurrent.CountDownLatch(1);
        var release = new java.util.concurrent.CountDownLatch(1);
        java.util.List<String> writes = new java.util.ArrayList<>();
        RecordingPigeonController controller = new RecordingPigeonController() {
            @Override public boolean setYawDegrees(double degrees) {
                if (degrees == 90.0) {
                    entered.countDown();
                    try { release.await(); }
                    catch (InterruptedException exception) { Thread.currentThread().interrupt(); throw new IllegalStateException(exception); }
                }
                writes.add("yaw:" + degrees);
                return super.setYawDegrees(degrees);
            }
            @Override public void reset() { writes.add("reset"); super.reset(); }
        };
        try (CtrePigeon2Handle handle = new CtrePigeon2Handle(ImuDevice.of(ImuKinds.PIGEON_2, 96), controller);
                CtrePigeon2Handle healthy = new CtrePigeon2Handle(ImuDevice.of(ImuKinds.PIGEON_2, 97),
                        new RecordingPigeonController())) {
            handle.refreshInputs();
            assertEquals(12.0, handle.yawDegrees());
            handle.setYawDegrees(90.0);
            assertTrue(entered.await(1, java.util.concurrent.TimeUnit.SECONDS));
            handle.setYawDegrees(45.0);
            handle.reset();
            assertFalse(handle.configurationStatus().ready());
            assertFalse(handle.isConnected());
            assertTrue(handle.isCalibrating());
            assertEquals(12.0, handle.yawDegrees(), "the pre-mutation snapshot is retained until acknowledgement");
            assertEquals(12.0, healthy.yawDegrees(), "healthy reads do not wait behind another IMU mutation");
            release.countDown();
            awaitConfiguration(handle);
            assertEquals(java.util.List.of("yaw:90.0", "reset"), writes);
            int sampledYawCalls = controller.yawCalls;
            assertEquals(0.0, handle.yawDegrees());
            assertEquals(sampledYawCalls, controller.yawCalls,
                    "acknowledged yaw is available immediately without a native refresh");
            assertTrue(handle.isConnected());
            handle.zeroYaw();
            awaitConfiguration(handle);
            assertEquals("yaw:0.0", writes.get(writes.size() - 1));
        } finally { release.countDown(); }
    }

    private static class RecordingPigeonController implements CtrePigeon2Handle.Pigeon2Controller {
        private double yaw = 12.0;
        private int yawCalls;
        private int pitchCalls;
        private int rollCalls;

        @Override
        public double yawDegrees() {
            yawCalls++;
            return yaw;
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
            yaw = yawDegrees;
            return true;
        }

        @Override
        public void reset() { yaw = 0.0; }

        @Override
        public void close() {}
    }
}
