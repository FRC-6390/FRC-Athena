package ca.frc6390.athena.vendor.rev;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.Units;
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
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.HardwareBus;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import com.revrobotics.config.BaseConfig;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkParameters;
import java.lang.reflect.Field;
import java.util.Map;
import org.junit.jupiter.api.Test;

class RevBackendTest {
    private static ca.frc6390.athena.hardware.backend.ConfigurationStatus configuration(Object handle) {
        return handle instanceof ca.frc6390.athena.hardware.backend.MotorHandle motor
                ? motor.configurationStatus()
                : ((ca.frc6390.athena.hardware.backend.EncoderHandle) handle).configurationStatus();
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
    void repeatedOpenLoopCommandsSkipTheVendorCall() {
        RecordingSparkController controller = new RecordingSparkController();
        RevMotorHandle handle = new RevMotorHandle(MotorDevice.of(MotorKinds.NEO, 19), new RevMotorOptions(), controller);

        handle.setVoltage(2.0);
        handle.setVoltage(2.0);
        handle.setVoltage(2.0);
        assertEquals(1, controller.setVoltageCalls);
        handle.setVoltage(2.5);
        assertEquals(2, controller.setVoltageCalls);
        assertEquals(2.5, controller.lastVolts);

        handle.stop();
        handle.setVoltage(2.5);
        assertEquals(3, controller.setVoltageCalls, "stop() must not leave a stale voltage cached");
        handle.setPercentOutput(0.5);
        handle.setPercentOutput(0.5);
        assertEquals(1, controller.setPercentCalls);
        handle.setVoltage(2.5);
        assertEquals(4, controller.setVoltageCalls, "switching modes resends");
    }

    @Test
    void runtimeMotorConfigurationIsAppliedWithoutReplacingTheDeclaration() {
        RecordingSparkController controller = new RecordingSparkController();
        MotorDevice declaration = MotorDevice.of(MotorKinds.NEO, 18).coast();
        RevMotorHandle handle = new RevMotorHandle(declaration, new RevMotorOptions(), controller);
        MotorRuntimeConfig runtime = new MotorRuntimeConfig(MotorNeutralMode.BRAKE, true, 31, 42);

        handle.applyRuntimeConfiguration(runtime);

        awaitConfiguration(handle);

        assertTrue(handle.supportsRuntimeConfiguration());
        assertEquals(MotorNeutralMode.COAST, handle.device().neutralMode());
        assertEquals(runtime, controller.runtimeConfiguration);
    }

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
                .smartCurrentLimit(Amps.of(-1))
                .openLoopRamp(Seconds.of(Double.NaN))
                .closedLoopRamp(Seconds.of(1.25));

        assertEquals(0, options.smartCurrentLimit().in(Amps));
        assertEquals(0.0, options.openLoopRamp().in(Seconds));
        assertEquals(1.25, options.closedLoopRamp().in(Seconds));
        assertFalse(options.resetSafeParameters());
        assertTrue(options.persistParameters());
    }

    @Test
    void undeclaredBrushlessCurrentLimitAppliesSafeDefault() throws Exception {
        Map<Integer, Object> neo = parameters(RevMotorHandle.activationConfig(
                MotorDevice.of(MotorKinds.NEO, 2), new RevMotorOptions(), false));
        Map<Integer, Object> neo550 = parameters(RevMotorHandle.activationConfig(
                MotorDevice.of(MotorKinds.NEO_550, 3), new RevMotorOptions(), false));
        Map<Integer, Object> vortex = parameters(RevMotorHandle.activationConfig(
                MotorDevice.of(MotorKinds.NEO_VORTEX, 4), new RevMotorOptions(), true));
        Map<Integer, Object> cim = parameters(RevMotorHandle.activationConfig(
                MotorDevice.of(MotorKinds.CIM, 5), new RevMotorOptions(), false));

        assertEquals(RevMotorHandle.DEFAULT_NEO_CURRENT_LIMIT_AMPS,
                neo.get(SparkParameters.kSmartCurrentStallLimit.value));
        assertEquals(RevMotorHandle.DEFAULT_NEO_550_CURRENT_LIMIT_AMPS,
                neo550.get(SparkParameters.kSmartCurrentStallLimit.value));
        assertEquals(RevMotorHandle.DEFAULT_NEO_VORTEX_CURRENT_LIMIT_AMPS,
                vortex.get(SparkParameters.kSmartCurrentStallLimit.value));
        assertFalse(cim.containsKey(SparkParameters.kSmartCurrentStallLimit.value));
        assertFalse(cim.containsKey(SparkParameters.kCurrentChop.value));
    }

    @Test
    void failedActivationRetainsOneGatedSparkForRetryAndReleasesItOnClose() {
        RecordingSparkController controller = new RecordingSparkController();
        controller.configureResult = false;
        RevMotorHandle handle = new RevMotorHandle(MotorDevice.of(MotorKinds.NEO, 1), null, controller);

        handle.activate();
        awaitFailure(handle);
        assertFalse(controller.closed);
        handle.close();
        awaitCondition(() -> controller.closed);

        RecordingSparkController throwing = new RecordingSparkController();
        throwing.configureFailure = new IllegalStateException("Invalid parameter");
        RevMotorHandle rejected = new RevMotorHandle(MotorDevice.of(MotorKinds.NEO, 2), null, throwing);
        rejected.activate();
        awaitFailure(rejected);
        assertTrue(rejected.configurationStatus().error().contains("Invalid parameter"));
        rejected.close();
        awaitCondition(() -> throwing.closed);
    }

    @Test
    void hardwareFollowerIsConfiguredOnceByFollow() {
        RecordingSparkController leaderController = new RecordingSparkController();
        RecordingSparkController followerController = new RecordingSparkController();
        MotorDevice leaderDevice = MotorDevice.of(MotorKinds.NEO, 1);
        RevMotorHandle leader = new RevMotorHandle(leaderDevice, null, leaderController);
        RevMotorHandle follower = new RevMotorHandle(
                MotorDevice.of(MotorKinds.NEO, 2).follow(leaderDevice), null, followerController);

        follower.activate();

        awaitConfiguration(follower);
        follower.follow(leader, false);
        awaitConfiguration(follower);

        assertEquals(0, followerController.configureCalls);
        assertEquals(1, followerController.followLeaderId);
    }

    @Test
    void preservedDeviceCurrentLimitLeavesHardwareClientValue() throws Exception {
        RevMotorOptions options = new RevMotorOptions().preserveDeviceCurrentLimit(true);
        SparkBaseConfig config = RevMotorHandle.activationConfig(
                MotorDevice.of(MotorKinds.NEO, 2), options, false);
        Map<Integer, Object> motorParameters = parameters(config);

        assertFalse(options.resetSafeParameters());
        assertFalse(motorParameters.containsKey(SparkParameters.kSmartCurrentStallLimit.value));
        assertFalse(motorParameters.containsKey(SparkParameters.kSmartCurrentFreeLimit.value));
        assertFalse(motorParameters.containsKey(SparkParameters.kSmartCurrentConfig.value));
        assertFalse(motorParameters.containsKey(SparkParameters.kCurrentChop.value));
        assertFalse(motorParameters.containsKey(SparkParameters.kOpenLoopRampRate.value));
        assertFalse(motorParameters.containsKey(SparkParameters.kClosedLoopRampRate.value));
        assertFalse(motorParameters.containsKey(SparkParameters.kCompensatedNominalVoltage.value));
        assertTrue(parameters(config.softLimit).isEmpty());
        assertTrue(parameters(config.limitSwitch).isEmpty());
    }

    @Test
    void realRevConfigUsesSmartLimitForNeoAndSecondaryLimitForBrushedMotor() throws Exception {
        SparkBaseConfig neo = RevMotorHandle.activationConfig(
                MotorDevice.of(MotorKinds.NEO, 2).currentLimit(Units.Amps.of(35)),
                new RevMotorOptions(),
                false);
        SparkBaseConfig cim = RevMotorHandle.activationConfig(
                MotorDevice.of(MotorKinds.CIM, 3).currentLimit(Units.Amps.of(30)),
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
        assertFalse(cimParameters.containsKey(SparkParameters.kOpenLoopRampRate.value));
        assertFalse(cimParameters.containsKey(SparkParameters.kClosedLoopRampRate.value));
    }

    @Test
    void realRevConfigIncludesSafetyTelemetryAndAbsoluteEncoderCalibration() throws Exception {
        RevMotorOptions motorOptions = new RevMotorOptions()
                .voltageCompensation(Volts.of(12.0))
                .telemetrySignalPeriod(Milliseconds.of(25))
                .forwardSoftLimit(Rotations.of(20.5))
                .reverseSoftLimit(Rotations.of(-1.5))
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
                        .absoluteEncoderSignalPeriod(Milliseconds.of(30)),
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
        RevMotorOptions options = new RevMotorOptions().smartCurrentLimit(Amps.of(35));
        RevMotorHandle handle = new RevMotorHandle(
                MotorDevice.of(MotorKinds.NEO, 1).neutralMode(MotorNeutralMode.BRAKE),
                options,
                controller);

        assertEquals(0, controller.configureCalls);

        handle.activate();

        awaitConfiguration(handle);
        handle.activate();
        awaitConfiguration(handle);

        assertEquals(1, controller.configureCalls);
        assertEquals(options, controller.options);
        assertEquals(MotorNeutralMode.BRAKE, controller.device.neutralMode());
    }

    @Test
    void explicitSupplyIsIndependentOfRevPhaseCurrentLimit() {
        assertEquals(RevMotorHandle.DEFAULT_NEO_CURRENT_LIMIT_AMPS, RevMotorHandle.effectiveSmartCurrentLimit(
                MotorDevice.of(MotorKinds.NEO, 1).supplyCurrentLimit(Units.Amps.of(35)),
                new RevMotorOptions()));
        assertEquals(60, RevMotorHandle.effectiveSmartCurrentLimit(
                MotorDevice.of(MotorKinds.NEO, 2).statorCurrentLimit(Units.Amps.of(60)),
                new RevMotorOptions()));
        assertEquals(60, RevMotorHandle.effectiveSmartCurrentLimit(
                MotorDevice.of(MotorKinds.NEO, 3).currentLimit(Units.Amps.of(0)).statorCurrentLimit(Units.Amps.of(60)),
                new RevMotorOptions()));
        assertEquals(50, RevMotorHandle.effectiveSmartCurrentLimit(
                MotorDevice.of(MotorKinds.NEO, 4).supplyCurrentLimit(Units.Amps.of(70)).statorCurrentLimit(Units.Amps.of(50)),
                new RevMotorOptions()));
        assertEquals(55, RevMotorHandle.effectiveSmartCurrentLimit(
                MotorDevice.of(MotorKinds.NEO, 5).supplyCurrentLimit(Units.Amps.of(35)).statorCurrentLimit(Units.Amps.of(30)),
                new RevMotorOptions().smartCurrentLimit(Amps.of(55))));
        assertEquals(60, RevMotorHandle.effectiveSmartCurrentLimit(
                MotorDevice.of(MotorKinds.NEO, 6).supplyCurrentLimit(Amps.of(10)).statorCurrentLimit(Amps.of(60)),
                new RevMotorOptions()));
    }

    @Test
    void activationAndRuntimeConfigurePhaseOnlyWhileSupplyIsSoftwareEnforced() throws Exception {
        MotorDevice motor = MotorDevice.of(MotorKinds.NEO, 7)
                .supplyCurrentLimit(Amps.of(10)).statorCurrentLimit(Amps.of(60));
        Map<Integer, Object> activation = parameters(RevMotorHandle.activationConfig(motor, new RevMotorOptions(), false));
        Map<Integer, Object> runtime = parameters(RevMotorHandle.runtimeConfig(
                new MotorRuntimeConfig(MotorNeutralMode.BRAKE, false, 10, 60), true, false));
        assertEquals(60, activation.get(SparkParameters.kSmartCurrentStallLimit.value));
        assertEquals(60, runtime.get(SparkParameters.kSmartCurrentStallLimit.value));
        Map<Integer, Object> supplyOnly = parameters(RevMotorHandle.runtimeConfig(
                new MotorRuntimeConfig(MotorNeutralMode.BRAKE, false, 10, 0), true, false));
        assertEquals(RevMotorHandle.DEFAULT_NEO_CURRENT_LIMIT_AMPS,
                supplyOnly.get(SparkParameters.kSmartCurrentStallLimit.value));
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

        awaitConfiguration(follower);

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
                    .currentLimit(Units.Amps.of(35));
            RevMotorHandle follower = new RevMotorHandle(
                    followerDevice, null, followerControllers[index]);

            follower.activate();

            awaitConfiguration(follower);
            follower.follow(leader, false);
            awaitConfiguration(follower);
        }

        for (RecordingSparkController controller : followerControllers) {
            assertEquals(1, controller.followLeaderId);
            assertFalse(controller.followInverted);
            assertEquals(MotorNeutralMode.BRAKE, controller.followDevice.neutralMode());
            assertEquals(35, controller.followDevice.currentLimit().in(edu.wpi.first.units.Units.Amps), 0.0);
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
        awaitConfiguration(handle);

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

        awaitConfiguration(handle);

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

        awaitConfiguration(handle);
        handle.refreshInputs();
        assertEquals(1, controller.enableAbsoluteEncoderCalls);
        assertEquals(1, controller.absolutePositionReads);
        assertEquals(1, controller.absoluteVelocityReads);

        handle.close();
        awaitCondition(() -> controller.closed);
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

    @Test
    void faultedReportsTheFatalFaultCapturedByTheLastRefresh() {
        RecordingSparkController controller = new RecordingSparkController();
        RevMotorHandle handle = new RevMotorHandle(
                MotorDevice.of(MotorKinds.NEO, 2), new RevMotorOptions(), controller);

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
    void fatalFaultMaskCoversOnlyFaultsThatStopTheMotor() {
        SparkBase.Faults fatal = new SparkBase.Faults(RevMotorHandle.FATAL_FAULT_BITS);
        assertTrue(fatal.gateDriver);
        assertTrue(fatal.motorType);
        assertTrue(fatal.escEeprom);
        assertTrue(fatal.firmware);
        assertFalse(fatal.other);
        assertFalse(fatal.sensor);
        assertFalse(fatal.can);
        assertFalse(fatal.temperature);
    }

    @Test
    void promotionAndRejoinWaitForTheOrderedFollowerConfigurationWithoutHoldingTheGraph() throws Exception {
        var promotionEntered = new java.util.concurrent.CountDownLatch(1);
        var promotionRelease = new java.util.concurrent.CountDownLatch(1);
        var rejoinEntered = new java.util.concurrent.CountDownLatch(1);
        var rejoinRelease = new java.util.concurrent.CountDownLatch(1);
        RecordingSparkController original = new RecordingSparkController() {
            @Override public boolean follow(MotorDevice device, RevMotorOptions options, int id, boolean inverted) {
                rejoinEntered.countDown();
                waitFor(rejoinRelease);
                return super.follow(device, options, id, inverted);
            }
        };
        RecordingSparkController promoted = new RecordingSparkController() {
            @Override public boolean stopFollowing(MotorDevice device, RevMotorOptions options) {
                promotionEntered.countDown();
                waitFor(promotionRelease);
                return true;
            }
        };
        MotorDevice leader = MotorDevice.of(MotorKinds.NEO, 92);
        MotorDevice follower = MotorDevice.of(MotorKinds.NEO, 93).follow(leader);
        var backend = new ca.frc6390.athena.hardware.backend.MotorBackend() {
            @Override public boolean supports(MotorKind kind) { return true; }
            @Override public boolean supportsHardwareFollowing(MotorDevice a, MotorDevice b) { return true; }
            @Override public ca.frc6390.athena.hardware.backend.MotorHandle create(MotorDevice device) {
                return new RevMotorHandle(device, null, device.id() == leader.id() ? original : promoted);
            }
        };
        try (var graph = ca.frc6390.athena.hardware.runtime.HardwareGraph.using(
                ca.frc6390.athena.hardware.backend.BackendRegistry.of(backend))) {
            var group = graph.motorGroup(new ca.frc6390.athena.hardware.runtime.MotorGroupPolicy(
                    leader, java.util.List.of(follower), true, true, 1, 1.0, 1.0, true, 1));
            var route = graph.motor(leader);
            var followerHandle = graph.motor(follower);
            awaitConfiguration(followerHandle);
            awaitConfiguration(route);
            graph.refreshInputs();
            original.fatalFault = true;
            graph.refreshInputs();
            assertTrue(promotionEntered.await(1, java.util.concurrent.TimeUnit.SECONDS));
            assertEquals(follower, group.leader());
            assertFalse(route.configurationStatus().ready());
            int before = promoted.setVoltageCalls;
            route.setVoltage(6.0);
            assertEquals(before, promoted.setVoltageCalls, "pending stop-following cannot accept drive output");
            assertEquals(route, graph.motor(leader), "lookups remain available while the driver is blocked");
            promotionRelease.countDown();
            awaitConfiguration(route);
            route.setVoltage(6.0);
            assertEquals(6.0, promoted.lastVolts);

            original.fatalFault = false;
            graph.refreshInputs();
            assertTrue(rejoinEntered.await(1, java.util.concurrent.TimeUnit.SECONDS));
            assertFalse(route.configurationStatus().ready(), "the group waits until rejoin routing is applied");
            rejoinRelease.countDown();
            awaitConfiguration(route);
            assertEquals(follower.id(), original.followLeaderId);
            route.setVoltage(8.0);
            assertEquals(8.0, promoted.lastVolts);
        } finally {
            promotionRelease.countDown();
            rejoinRelease.countDown();
        }
    }

    private static void waitFor(java.util.concurrent.CountDownLatch latch) {
        try { latch.await(); }
        catch (InterruptedException exception) { Thread.currentThread().interrupt(); throw new IllegalStateException(exception); }
    }

    private static class RecordingSparkController implements RevMotorHandle.SparkController {
        private int configureCalls;
        private boolean configureResult = true;
        private RuntimeException configureFailure;
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
        private MotorRuntimeConfig runtimeConfiguration;
        private boolean fatalFault;
        private int fatalFaultReads;

        @Override
        public boolean fatalFault() {
            fatalFaultReads++;
            return fatalFault;
        }

        @Override
        public boolean configure(MotorDevice device, RevMotorOptions options) {
            configureCalls++;
            this.device = device;
            this.options = options;
            if (configureFailure != null) {
                throw configureFailure;
            }
            return configureResult;
        }

        @Override
        public boolean configureRuntime(MotorRuntimeConfig configuration, boolean brushless) {
            runtimeConfiguration = configuration;
            return true;
        }

        private int setPercentCalls;
        private int setVoltageCalls;
        private double lastVolts = Double.NaN;

        @Override
        public void setPercent(double percent) {
            setPercentCalls++;
        }

        @Override
        public void setVoltage(double volts) {
            setVoltageCalls++;
            lastVolts = volts;
        }

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
