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
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.HardwareBus;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkParameters;
import java.lang.reflect.Field;
import java.util.Map;
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
        assertTrue(backend.supports(HardwareBus.rio()
                .dio(1).encoder(EncoderKinds.REV_THROUGH_BORE_V2)));
        assertTrue(backend.supports(HardwareBus.rio()
                .quadrature(2, 3).encoder(EncoderKinds.REV_THROUGH_BORE_QUADRATURE)));
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
        assertTrue(options.resetSafeParameters());
        assertTrue(options.persistParameters());
    }

    @Test
    void realRevConfigUsesSmartLimitForNeoAndSecondaryLimitForBrushedMotor() throws Exception {
        SparkBaseConfig neo = RevMotorHandle.activationConfig(
                MotorDevice.of(MotorKinds.NEO, 2).currentLimit(35),
                new RevMotorOptions(),
                false);
        SparkBaseConfig cim = RevMotorHandle.activationConfig(
                MotorDevice.of(MotorKinds.CIM, 3).currentLimit(30),
                new RevMotorOptions().primaryEncoderCountsPerRevolution(8192),
                false);

        Map<Integer, Object> neoParameters = parameters(neo);
        Map<Integer, Object> cimParameters = parameters(cim);
        assertEquals(35, neoParameters.get(SparkParameters.kSmartCurrentStallLimit.value));
        assertFalse(neoParameters.containsKey(SparkParameters.kCurrentChop.value));
        assertEquals(0, neoParameters.get(SparkParameters.kFollowerModeLeaderId.value));
        assertEquals(30.0f, cimParameters.get(SparkParameters.kCurrentChop.value));
        assertFalse(cimParameters.containsKey(SparkParameters.kSmartCurrentStallLimit.value));
        assertEquals(8192, parameters(cim.encoder).get(SparkParameters.kEncoderCountsPerRev.value));
        assertEquals(0.0f, cimParameters.get(SparkParameters.kOpenLoopRampRate.value));
        assertEquals(0.0f, cimParameters.get(SparkParameters.kClosedLoopRampRate.value));
    }

    @Test
    void realRevConfigIncludesSafetyTelemetryAndAbsoluteEncoderCalibration() throws Exception {
        RevMotorOptions motorOptions = new RevMotorOptions()
                .voltageCompensation(12.0)
                .telemetrySignalPeriodMs(25)
                .forwardSoftLimitRotations(20.5)
                .reverseSoftLimitRotations(-1.5)
                .forwardLimitSwitch(true, true)
                .reverseLimitSwitch(false, false);
        SparkBaseConfig motor = RevMotorHandle.activationConfig(
                MotorDevice.of(MotorKinds.NEO, 8), motorOptions, false);
        Map<Integer, Object> motorParameters = parameters(motor);
        Map<Integer, Object> softLimitParameters = parameters(motor.softLimit);
        Map<Integer, Object> limitSwitchParameters = parameters(motor.limitSwitch);
        Map<Integer, Object> signalParameters = parameters(motor.signals);

        assertEquals(12.0f, motorParameters.get(SparkParameters.kCompensatedNominalVoltage.value));
        assertEquals(true, softLimitParameters.get(SparkParameters.kSoftLimitFwdEn.value));
        assertEquals(true, softLimitParameters.get(SparkParameters.kSoftLimitRevEn.value));
        assertEquals(20.5f, softLimitParameters.get(SparkParameters.kSoftLimitForward.value));
        assertEquals(-1.5f, softLimitParameters.get(SparkParameters.kSoftLimitReverse.value));
        assertEquals(25, signalParameters.get(SparkParameters.kStatus0Period.value));
        assertEquals(1, limitSwitchParameters.get(SparkParameters.kHardLimitFwdEn.value));
        assertEquals(0, limitSwitchParameters.get(SparkParameters.kHardLimitRevEn.value));

        SparkBaseConfig absolute = RevMotorHandle.absoluteEncoderConfig(
                new RevMotorOptions()
                        .absoluteEncoderProfile(RevMotorOptions.AbsoluteEncoderProfile.REV_THROUGH_BORE_V2)
                        .absoluteEncoderAverageDepth(8)
                        .absoluteEncoderSignalPeriodMs(30),
                false);
        Map<Integer, Object> absoluteParameters = parameters(absolute.absoluteEncoder);
        Map<Integer, Object> absoluteSignals = parameters(absolute.signals);
        assertEquals(3.884f, absoluteParameters.get(SparkParameters.kDutyCycleEncoderStartPulseUs.value));
        assertEquals(998.06f, absoluteParameters.get(SparkParameters.kDutyCycleEncoderEndPulseUs.value));
        assertEquals(3, absoluteParameters.get(SparkParameters.kDutyCycleAverageDepth.value));
        assertEquals(30, absoluteSignals.get(SparkParameters.kStatus5Period.value));
    }

    @Test
    void nonRioCanBusFailsInsteadOfSilentlyColliding() {
        MotorDevice motor = MotorDevice.of(MotorKinds.NEO, 1).canbus("canivore");

        IllegalArgumentException failure = assertThrows(
                IllegalArgumentException.class,
                () -> new RevMotorHandle(motor, new RevMotorOptions()));

        assertTrue(failure.getMessage().contains("roboRIO CAN bus"));
        assertTrue(failure.getMessage().contains("canivore"));
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

        handle.refreshInputs();
        assertEquals(0.42, handle.absolutePositionRotations(), 1.0e-9);
        assertEquals(0.42, handle.positionRotations(), 1.0e-9);
        assertThrows(UnsupportedOperationException.class, handle::velocityRotationsPerSecond);
    }

    @Test
    void disconnectedThroughBoreFailsRefreshClearly() {
        RevThroughBoreEncoderHandle handle = new RevThroughBoreEncoderHandle(
                HardwareBus.rio().dio(4).encoder(EncoderKinds.REV_THROUGH_BORE),
                new RevThroughBoreEncoderHandle.ThroughBoreController() {
                    @Override public double absolutePositionRotations() { return 0.0; }
                    @Override public double velocityRotationsPerSecond() { return 0.0; }
                    @Override public boolean isConnected() { return false; }
                });

        IllegalStateException failure = assertThrows(IllegalStateException.class, handle::refreshInputs);

        assertTrue(failure.getMessage().contains("disconnected"));
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
    void ctreOrientedCurrentLimitsMapConservativelyToRevSmartCurrentLimit() {
        assertEquals(35, RevMotorHandle.effectiveSmartCurrentLimit(
                MotorDevice.of(MotorKinds.NEO, 1).supplyCurrentLimit(35),
                new RevMotorOptions()));
        assertEquals(40, RevMotorHandle.effectiveSmartCurrentLimit(
                MotorDevice.of(MotorKinds.NEO, 2).statorCurrentLimit(60),
                new RevMotorOptions()));
        assertEquals(60, RevMotorHandle.effectiveSmartCurrentLimit(
                MotorDevice.of(MotorKinds.NEO, 3).currentLimit(0).statorCurrentLimit(60),
                new RevMotorOptions()));
        assertEquals(50, RevMotorHandle.effectiveSmartCurrentLimit(
                MotorDevice.of(MotorKinds.NEO, 4).supplyCurrentLimit(70).statorCurrentLimit(50),
                new RevMotorOptions()));
        assertEquals(55, RevMotorHandle.effectiveSmartCurrentLimit(
                MotorDevice.of(MotorKinds.NEO, 5).supplyCurrentLimit(35).statorCurrentLimit(30),
                new RevMotorOptions().smartCurrentLimit(55)));
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
    void fourNeoFollowersCanShareOneLeader() {
        RevMotorHandle leader = new RevMotorHandle(
                MotorDevice.of(MotorKinds.NEO, 1), null, new RecordingSparkController());
        RecordingSparkController[] followerControllers = {
            new RecordingSparkController(),
            new RecordingSparkController(),
            new RecordingSparkController(),
            new RecordingSparkController()
        };

        for (int index = 0; index < followerControllers.length; index++) {
            MotorDevice followerDevice = MotorDevice.of(MotorKinds.NEO, index + 2)
                    .brake()
                    .currentLimit(35);
            RevMotorHandle follower = new RevMotorHandle(
                    followerDevice, null, followerControllers[index]);

            follower.activate();
            follower.follow(leader, false);
        }

        for (RecordingSparkController controller : followerControllers) {
            assertEquals(1, controller.followLeaderId);
            assertFalse(controller.followInverted);
            assertEquals(MotorNeutralMode.BRAKE, controller.followDevice.neutralMode());
            assertEquals(35, controller.followDevice.currentLimitAmps());
        }
    }

    @Test
    void motorDoesNotAdvertiseDutyCyclePidAsVoltageClosedLoop() {
        RevMotorHandle handle = new RevMotorHandle(
                MotorDevice.of(MotorKinds.NEO, 1),
                new RevMotorOptions(),
                new RecordingSparkController());

        assertEquals(MotorControlCapabilities.OPEN_LOOP_ONLY, handle.controlCapabilities());
    }

    @Test
    void motorSensorReadsAreSnapshottedUntilRefresh() {
        RecordingSparkController controller = new RecordingSparkController();
        RevMotorHandle handle = new RevMotorHandle(
                MotorDevice.of(MotorKinds.NEO, 2),
                new RevMotorOptions(),
                controller);
        handle.enableAbsoluteEncoder();

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
    void absoluteEncoderIsOnlyReadAfterItIsDeclaredAndHandleClosesController() {
        RecordingSparkController controller = new RecordingSparkController();
        RevMotorHandle handle = new RevMotorHandle(
                MotorDevice.of(MotorKinds.NEO, 2), new RevMotorOptions(), controller);

        handle.refreshInputs();
        assertEquals(0, controller.absolutePositionReads);
        assertEquals(0, controller.absoluteVelocityReads);

        handle.enableAbsoluteEncoder();
        handle.refreshInputs();
        assertEquals(1, controller.enableAbsoluteEncoderCalls);
        assertEquals(1, controller.absolutePositionReads);
        assertEquals(1, controller.absoluteVelocityReads);

        handle.close();
        assertTrue(controller.closed);
    }

    @Test
    void explicitVoltageSemanticClosedLoopRequestsAreRejected() {
        RecordingSparkController controller = new RecordingSparkController();
        RevMotorHandle handle = new RevMotorHandle(
                MotorDevice.of(MotorKinds.NEO, 2),
                new RevMotorOptions(),
                controller);
        MotorClosedLoopRequest request = MotorClosedLoopRequest.device(MotorClosedLoopConfig.empty());

        assertThrows(UnsupportedOperationException.class,
                () -> handle.setPositionTargetRotations(2.0, request));
        assertThrows(UnsupportedOperationException.class,
                () -> handle.setVelocityTargetRotationsPerSecond(3.0, request));
        assertEquals(MotorControlCapabilities.OPEN_LOOP_ONLY, handle.controlCapabilities());
    }

    private static final class RecordingSparkController implements RevMotorHandle.SparkController {
        private int configureCalls;
        private MotorDevice device;
        private RevMotorOptions options;
        private int positionReads;
        private int velocityReads;
        private int absolutePositionReads;
        private int absoluteVelocityReads;
        private double position;
        private double velocity;
        private double absolutePosition;
        private double absoluteVelocity;
        private int followLeaderId = -1;
        private boolean followInverted;
        private MotorDevice followDevice;
        private int enableAbsoluteEncoderCalls;
        private boolean closed;

        @Override
        public boolean configure(MotorDevice device, RevMotorOptions options) {
            configureCalls++;
            this.device = device;
            this.options = options;
            return true;
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
        public boolean setSensorPosition(double rotations) {
            position = rotations;
            return true;
        }

        @Override
        public boolean follow(
                MotorDevice device,
                RevMotorOptions options,
                int leaderId,
                boolean inverted) {
            followDevice = device;
            followLeaderId = leaderId;
            followInverted = inverted;
            return true;
        }

        @Override
        public boolean enableAbsoluteEncoder(RevMotorOptions options) {
            enableAbsoluteEncoderCalls++;
            return true;
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

        @Override
        public void close() {
            closed = true;
        }
    }

    @SuppressWarnings("unchecked")
    private static Map<Integer, Object> parameters(BaseConfig config) throws Exception {
        Field field = BaseConfig.class.getDeclaredField("parameters");
        field.setAccessible(true);
        return Map.copyOf((Map<Integer, Object>) field.get(config));
    }
}
