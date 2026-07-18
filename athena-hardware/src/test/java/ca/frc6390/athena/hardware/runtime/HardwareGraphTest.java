package ca.frc6390.athena.hardware.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.Future;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.TimeoutException;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.api.hardware.MotorControllerKinds;
import ca.frc6390.athena.api.FailurePolicy;
import ca.frc6390.athena.hardware.backend.BackendRegistry;
import ca.frc6390.athena.hardware.backend.EncoderBackend;
import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.backend.HardwareIdentity;
import ca.frc6390.athena.hardware.backend.ImuBackend;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.backend.MotorBackend;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.HardwareBus;
import ca.frc6390.athena.hardware.device.HardwareInterface;
import ca.frc6390.athena.hardware.device.HardwareAddress;
import ca.frc6390.athena.hardware.device.I2cPort;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.device.SpiPort;
import ca.frc6390.athena.hardware.sim.SimLimit;
import ca.frc6390.athena.hardware.sim.SimModel;

class HardwareGraphTest {
    @Test
    void hardwareIdentityIncludesCategoryKindBusIdAndIntegratedDetail() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 7).canbus("CANivore A");
        EncoderDevice encoder = motor.encoder();

        assertEquals("motor:ctre_talon_fx:canivore_a:7", HardwareIdentity.motor(motor).key());
        assertEquals("encoder:athena_integrated_motor:canivore_a:7:integrated", HardwareIdentity.encoder(encoder).key());
        assertEquals("encoder:athena_integrated_motor:canivore_a:7:absolute", HardwareIdentity.encoder(motor.absoluteEncoder()).key());
        assertEquals("imu:ctre_pigeon_2:rio:2", HardwareIdentity.imu(ImuDevice.of(ImuKinds.PIGEON_2, 2)).key());
    }

    @Test
    void hardwareBusAllowsExplicitControllerMotorPairing() {
        MotorDevice motor = HardwareBus.rio().motor(MotorControllerKinds.SPARK_FLEX, MotorKinds.NEO, 8);

        assertEquals(MotorControllerKinds.SPARK_FLEX, motor.kind().controllerKind());
        assertEquals(MotorKinds.NEO, motor.kind().motorKind());
        assertEquals("rev:spark-flex/neo", motor.kind().key());
    }

    @Test
    void graphCachesHandlesAndLooksUpMatchingBackends() {
        FakeMotorBackend motorBackend = new FakeMotorBackend();
        FakeEncoderBackend encoderBackend = new FakeEncoderBackend();
        FakeImuBackend imuBackend = new FakeImuBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(
                List.of(motorBackend),
                List.of(encoderBackend),
                List.of(imuBackend)));
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 2);
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 3);

        assertSame(graph.motor(motor), graph.motor(motor));
        assertSame(graph.encoder(encoder), graph.encoder(encoder));
        assertSame(graph.imu(imu), graph.imu(imu));
        assertEquals(1, motorBackend.created);
        assertEquals(1, encoderBackend.created);
        assertEquals(1, imuBackend.created);
        assertEquals(1, motorBackend.last.activateCalls);
        assertEquals(1, encoderBackend.last.activateCalls);
        assertEquals(1, imuBackend.last.activateCalls);

        graph.close();

        assertEquals(1, motorBackend.last.closeCalls);
        assertEquals(1, encoderBackend.last.closeCalls);
        assertEquals(1, imuBackend.last.closeCalls);
    }

    @Test
    void valueIdenticalMotorDeclarationsAreEachRuntimeBound() {
        FakeMotorBackend backend = new FakeMotorBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(backend));
        MotorDevice first = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        MotorDevice second = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);

        FakeMotorHandle handle = assertInstanceOf(FakeMotorHandle.class, graph.motor(first));
        handle.position = 4.25;

        assertSame(handle, graph.motor(second));
        assertEquals(4.25, second.positionRotations(), 1.0e-9);
        assertEquals(1, backend.created);
    }

    @Test
    void transientCreationFailuresRetryAndRebindDeclarations() {
        FakeMotorBackend motorBackend = new FakeMotorBackend();
        FakeEncoderBackend encoderBackend = new FakeEncoderBackend();
        FakeImuBackend imuBackend = new FakeImuBackend();
        motorBackend.failuresRemaining = 1;
        encoderBackend.failuresRemaining = 1;
        imuBackend.failuresRemaining = 1;
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(
                List.of(motorBackend),
                List.of(encoderBackend),
                List.of(imuBackend)));
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 11);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 12);
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 13);

        assertEquals(0.0, graph.motor(motor).integratedPositionRotations(), 1.0e-9);
        assertEquals(0.0, graph.encoder(encoder).positionRotations(), 1.0e-9);
        assertEquals(0.0, graph.imu(imu).yawDegrees(), 1.0e-9);
        assertEquals(3, graph.bindingFailures().size());

        FakeMotorHandle recoveredMotor = assertInstanceOf(FakeMotorHandle.class, graph.motor(motor));
        graph.encoder(encoder);
        FakeEncoderHandle recoveredEncoder = encoderBackend.last;
        FakeImuHandle recoveredImu = assertInstanceOf(FakeImuHandle.class, graph.imu(imu));
        recoveredMotor.position = 1.25;
        recoveredEncoder.position = 2.5;
        recoveredImu.yaw = 37.0;

        assertEquals(1.25, motor.positionRotations(), 1.0e-9);
        assertEquals(2.5, encoder.position(), 1.0e-9);
        assertEquals(37.0, imu.yawDegrees(), 1.0e-9);
        assertTrue(graph.bindingFailures().isEmpty());
        assertEquals(2, motorBackend.created);
        assertEquals(2, encoderBackend.created);
        assertEquals(2, imuBackend.created);
    }

    @Test
    void refreshInputsAutomaticallyRetriesFailedBindingsWithBackoff() {
        FakeEncoderBackend backend = new FakeEncoderBackend();
        backend.failuresRemaining = 2;
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(
                List.of(), List.of(backend), List.of()));
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 14);

        graph.encoder(encoder);
        graph.refreshInputs(0L);
        graph.refreshInputs(99_999_999L);

        assertEquals(2, backend.created);
        assertEquals(1, graph.bindingFailures().size());

        graph.refreshInputs(100_000_000L);
        backend.last.position = 0.625;

        assertEquals(3, backend.created);
        assertEquals(0.625, encoder.position(), 1.0e-9);
        assertTrue(graph.bindingFailures().isEmpty());
    }

    @Test
    void graphRejectsConflictingConfigurationForSamePhysicalMotor() {
        FakeMotorBackend backend = new FakeMotorBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(backend));
        MotorDevice first = MotorDevice.of(MotorKinds.KRAKEN_X60, 1).brake().currentLimit(40);
        MotorDevice conflicting = MotorDevice.of(MotorKinds.KRAKEN_X60, 1).coast().currentLimit(60);

        graph.motor(first);
        IllegalStateException failure = assertThrows(IllegalStateException.class, () -> graph.motor(conflicting));

        assertTrue(failure.getMessage().contains("Conflicting motor declarations"));
        assertTrue(failure.getMessage().contains(HardwareIdentity.motor(first).key()));
    }

    @Test
    void graphConfiguresDeclaredHardwareFollowerAgainstCachedLeader() {
        FakeMotorBackend backend = new FakeMotorBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(backend));
        MotorDevice leader = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 2).follow(leader).inverted();

        FakeMotorHandle followerHandle = assertInstanceOf(FakeMotorHandle.class, graph.motor(follower));
        FakeMotorHandle leaderHandle = assertInstanceOf(FakeMotorHandle.class, graph.motor(leader));

        assertEquals(2, backend.created);
        assertSame(leaderHandle, followerHandle.followLeader);
        assertEquals(true, followerHandle.followInverted);
    }

    @Test
    void crossBusFollowerUsesAthenaCommandMirroringInsteadOfHardwareFollow() {
        FakeMotorBackend backend = new FakeMotorBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(backend));
        MotorDevice leader = MotorDevice.of(MotorKinds.KRAKEN_X60, 1).canbus("rio");
        MotorDevice follower = MotorDevice.of(MotorKinds.KRAKEN_X60, 2)
                .canbus("canivore")
                .follow(leader)
                .inverted();

        graph.motor(follower);
        ActionContext.SoftwareMotorFollower softwareFollower = graph.softwareFollowers(leader).get(0);
        FakeMotorHandle backendFollower = backend.last;

        assertEquals(follower, softwareFollower.device());
        assertEquals(null, backendFollower.followLeader);
        assertFalse(backendFollower.device().isInverted());
        assertEquals(null, backendFollower.device().follower());

        softwareFollower.handle().setPercentOutput(0.25);
        assertEquals(-0.25, backendFollower.percent, 1.0e-9);
        softwareFollower.handle().setVoltage(6.0);
        assertEquals(-6.0, backendFollower.voltage, 1.0e-9);
        softwareFollower.handle().setPositionTargetRotations(3.0);
        assertEquals(-3.0, backendFollower.positionTarget, 1.0e-9);
        softwareFollower.handle().setVelocityTargetRotationsPerSecond(4.0);
        assertEquals(-4.0, backendFollower.velocityTarget, 1.0e-9);
        softwareFollower.handle().stop();
        assertEquals(1, backendFollower.stopCalls);
    }

    @Test
    void refreshInputsRecordsFailuresAndContinuesRefreshingOtherHandles() {
        FakeMotorBackend motorBackend = new FakeMotorBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(List.of(motorBackend), List.of(), List.of()));
        FakeMotorHandle first = assertInstanceOf(FakeMotorHandle.class, graph.motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 1)));
        FakeMotorHandle second = assertInstanceOf(FakeMotorHandle.class, graph.motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 2)));
        first.failRefresh = true;

        graph.refreshInputs();

        assertEquals(1, first.refreshCalls);
        assertEquals(1, second.refreshCalls);
        assertEquals(1, graph.refreshFailures().size());
        assertEquals(HardwareIdentity.motor(first.device()), graph.refreshFailures().get(0).identity());
    }

    @Test
    void failedRefreshesBackOffAndRecoverWithoutAnExceptionStorm() {
        FakeMotorBackend motorBackend = new FakeMotorBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(List.of(motorBackend), List.of(), List.of()));
        FakeMotorHandle motor = assertInstanceOf(
                FakeMotorHandle.class,
                graph.motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 1)));
        motor.failRefresh = true;

        graph.refreshInputs(1_000_000_000L);
        graph.refreshInputs(1_050_000_000L);

        assertEquals(1, motor.refreshCalls);
        assertEquals(1, graph.refreshFailures().size());

        motor.failRefresh = false;
        graph.refreshInputs(1_100_000_000L);

        assertEquals(2, motor.refreshCalls);
        assertTrue(graph.refreshFailures().isEmpty());
    }

    @Test
    void vendorRefreshRunsOutsideGraphMonitorAndCannotRunTwiceConcurrently() throws Exception {
        FakeMotorBackend motorBackend = new FakeMotorBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(motorBackend));
        FakeMotorHandle motor = assertInstanceOf(
                FakeMotorHandle.class,
                graph.motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 15)));
        motor.blockRefresh();
        ExecutorService executor = Executors.newFixedThreadPool(3);
        try {
            Future<?> firstRefresh = executor.submit(() -> graph.refreshInputs(1_000_000_000L));
            assertTrue(motor.refreshEntered.await(1, TimeUnit.SECONDS));

            Future<List<HardwareGraph.RefreshFailure>> graphAccess = executor.submit(graph::refreshFailures);
            assertTrue(graphAccess.get(1, TimeUnit.SECONDS).isEmpty());

            Future<?> overlappingRefresh = executor.submit(() -> graph.refreshInputs(1_000_000_000L));
            overlappingRefresh.get(1, TimeUnit.SECONDS);
            assertEquals(1, motor.refreshCalls);
            assertFalse(graph.cycleSnapshot().captured());

            motor.allowRefresh.countDown();
            firstRefresh.get(1, TimeUnit.SECONDS);
            assertEquals(1, motor.refreshCalls);
            assertTrue(graph.cycleSnapshot().captured());
        } finally {
            motor.releaseBlocks();
            executor.shutdownNow();
        }
    }

    @Test
    void snapshotHandleReadsRunOutsideGraphMonitor() throws Exception {
        FakeMotorBackend motorBackend = new FakeMotorBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(motorBackend));
        FakeMotorHandle motor = assertInstanceOf(
                FakeMotorHandle.class,
                graph.motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 16)));
        motor.blockPositionRead();
        ExecutorService executor = Executors.newFixedThreadPool(2);
        try {
            Future<?> refresh = executor.submit(() -> graph.refreshInputs(2_000_000_000L));
            assertTrue(motor.positionReadEntered.await(1, TimeUnit.SECONDS));

            Future<List<HardwareGraph.RefreshFailure>> graphAccess = executor.submit(graph::refreshFailures);
            assertTrue(graphAccess.get(1, TimeUnit.SECONDS).isEmpty());

            motor.allowPositionRead.countDown();
            refresh.get(1, TimeUnit.SECONDS);
            assertTrue(graph.cycleSnapshot().captured());
        } finally {
            motor.releaseBlocks();
            executor.shutdownNow();
        }
    }

    @Test
    void closeWaitsForInFlightVendorRefreshBeforeClosingHandle() throws Exception {
        FakeMotorBackend motorBackend = new FakeMotorBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(motorBackend));
        FakeMotorHandle motor = assertInstanceOf(
                FakeMotorHandle.class,
                graph.motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 17)));
        motor.blockRefresh();
        ExecutorService executor = Executors.newFixedThreadPool(2);
        try {
            Future<?> refresh = executor.submit(() -> graph.refreshInputs());
            assertTrue(motor.refreshEntered.await(1, TimeUnit.SECONDS));
            Future<?> close = executor.submit(graph::close);

            assertThrows(TimeoutException.class, () -> close.get(50, TimeUnit.MILLISECONDS));
            assertEquals(0, motor.closeCalls);

            motor.allowRefresh.countDown();
            refresh.get(1, TimeUnit.SECONDS);
            close.get(1, TimeUnit.SECONDS);
            assertEquals(1, motor.closeCalls);
        } finally {
            motor.releaseBlocks();
            executor.shutdownNow();
        }
    }

    @Test
    void integratedEncoderHandleReadsThroughCachedMotorHandle() {
        FakeMotorBackend motorBackend = new FakeMotorBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(List.of(motorBackend), List.of(), List.of()));
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 4);
        FakeMotorHandle motorHandle = assertInstanceOf(FakeMotorHandle.class, graph.motor(motor));
        motorHandle.position = 12.5;
        motorHandle.velocity = 3.25;

        EncoderHandle encoder = graph.encoder(motor.encoder());

        assertSame(encoder, graph.encoder(motor.encoder()));
        assertEquals(12.5, encoder.positionRotations(), 1.0e-9);
        assertEquals(3.25, encoder.velocityRotationsPerSecond(), 1.0e-9);
        assertEquals(1, motorBackend.created);
        assertEquals(1, motorHandle.enableIntegratedEncoderCalls);
    }

    @Test
    void encoderDeclarationReadsBoundRuntimeSnapshotDirectly() {
        FakeEncoderBackend backend = new FakeEncoderBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(List.of(), List.of(backend), List.of()));
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 6)
                .offset(0.25)
                .gearRatio(0.5)
                .conversion(360.0);
        graph.encoder(encoder);
        backend.last.position = 0.75;
        backend.last.absolutePosition = 0.5;
        backend.last.velocity = 2.0;

        assertEquals(90.0, encoder.position(), 1.0e-9);
        assertEquals(45.0, encoder.absolutePosition(), 1.0e-9);
        assertEquals(360.0, encoder.velocity(), 1.0e-9);

        graph.close();
        assertThrows(IllegalStateException.class, encoder::position);
    }

    @Test
    void sharedDeclarationRequiresScopeWhenBoundToMultipleRuntimes() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 7);
        FakeEncoderBackend firstBackend = new FakeEncoderBackend();
        FakeEncoderBackend secondBackend = new FakeEncoderBackend();
        HardwareGraph first = HardwareGraph.using(BackendRegistry.of(List.of(), List.of(firstBackend), List.of()));
        HardwareGraph second = HardwareGraph.using(BackendRegistry.of(List.of(), List.of(secondBackend), List.of()));
        first.encoder(encoder);
        second.encoder(encoder);
        firstBackend.last.position = 1.0;
        secondBackend.last.position = 2.0;

        assertThrows(IllegalStateException.class, encoder::position);
        assertEquals(1.0, first.runtimeScope().call(encoder::position), 1.0e-9);
        assertEquals(2.0, second.runtimeScope().call(encoder::position), 1.0e-9);

        first.close();
        assertEquals(2.0, encoder.position(), 1.0e-9);
        second.close();
    }

    @Test
    void imuDeclarationReadsBoundRuntimeSnapshotDirectly() {
        FakeImuBackend backend = new FakeImuBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(List.of(), List.of(), List.of(backend)));
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 8);
        graph.imu(imu);
        backend.last.yaw = 42.5;

        assertEquals(42.5, imu.yawDegrees(), 1.0e-9);

        graph.close();
        assertThrows(IllegalStateException.class, imu::yawDegrees);
    }

    @Test
    void readOnlyEncoderUsesSoftwareRelativePositionAdjustment() {
        FakeEncoderBackend encoderBackend = new FakeEncoderBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(
                List.of(), List.of(encoderBackend), List.of()));
        EncoderDevice device = EncoderDevice.of(EncoderKinds.CANCODER, 8);
        EncoderHandle encoder = graph.encoder(device);
        encoderBackend.last.position = 0.25;

        encoder.setPositionRotations(3.0);

        assertEquals(3.0, encoder.positionRotations(), 1.0e-9);
        encoderBackend.last.position = 0.75;
        assertEquals(3.5, encoder.positionRotations(), 1.0e-9);
    }

    @Test
    void motorAbsoluteEncoderHandleReadsThroughCachedMotorHandle() {
        FakeMotorBackend motorBackend = new FakeMotorBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(List.of(motorBackend), List.of(), List.of()));
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 5);
        FakeMotorHandle motorHandle = assertInstanceOf(FakeMotorHandle.class, graph.motor(motor));
        motorHandle.absolutePosition = 0.25;
        motorHandle.absoluteVelocity = 1.5;

        EncoderHandle encoder = graph.encoder(motor.absoluteEncoder());

        assertSame(encoder, graph.encoder(motor.absoluteEncoder()));
        assertEquals(0.25, encoder.positionRotations(), 1.0e-9);
        assertEquals(0.25, encoder.absolutePositionRotations(), 1.0e-9);
        assertEquals(1.5, encoder.velocityRotationsPerSecond(), 1.0e-9);
        assertEquals(1, motorBackend.created);
        assertEquals(1, motorHandle.enableAbsoluteEncoderCalls);
    }

    @Test
    void panicPolicyAndDefaultHandleMethodsFailFast() {
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(List.of(), List.of(), List.of()));

        assertThrows(IllegalStateException.class, () -> graph.motor(
                MotorDevice.of(MotorKinds.KRAKEN_X60, 1).failurePolicy(FailurePolicy.PANIC)));
        assertThrows(IllegalStateException.class, () -> graph.encoder(
                EncoderDevice.of(EncoderKinds.CANCODER, 1).failurePolicy(FailurePolicy.PANIC)));
        assertThrows(IllegalStateException.class, () -> graph.imu(
                ImuDevice.of(ImuKinds.PIGEON_2, 1).failurePolicy(FailurePolicy.PANIC)));

        MotorHandle defaultMotor = new MotorHandle() {
            @Override
            public MotorDevice device() {
                return MotorDevice.of(MotorKinds.KRAKEN_X60, 9);
            }
        };
        EncoderHandle defaultEncoder = new EncoderHandle() {
            @Override
            public EncoderDevice device() {
                return EncoderDevice.of(EncoderKinds.CANCODER, 9);
            }
        };

        assertThrows(UnsupportedOperationException.class, () -> defaultMotor.setPercentOutput(0.1));
        assertThrows(UnsupportedOperationException.class, defaultEncoder::positionRotations);
    }

    @Test
    void deviceValidationRejectsInvalidDeclarations() {
        assertThrows(NullPointerException.class, () -> MotorDevice.of(null, 1));
        assertThrows(IllegalArgumentException.class,
                () -> MotorDevice.of(MotorKinds.KRAKEN_X60, 1).currentLimit(-1));
        assertThrows(IllegalArgumentException.class,
                () -> MotorDevice.of(MotorKinds.KRAKEN_X60, 1).supplyCurrentLimit(-1));
        assertThrows(IllegalArgumentException.class,
                () -> MotorDevice.of(MotorKinds.KRAKEN_X60, 1).statorCurrentLimit(-1));
        assertThrows(NullPointerException.class, () -> EncoderDevice.of(null, 1));
        assertThrows(NullPointerException.class, () -> ImuDevice.of(null, 1));
        assertThrows(IllegalArgumentException.class, () -> EncoderDevice.of(EncoderKinds.CANCODER, 1).gearRatio(0.0));
        assertThrows(IllegalArgumentException.class, () -> EncoderDevice.of(EncoderKinds.CANCODER, 1).conversion(Double.NaN));
        EncoderDevice dioEncoder = HardwareBus.rio()
                .dio(2).encoder(EncoderKinds.REV_THROUGH_BORE);
        assertThrows(IllegalStateException.class, dioEncoder::id);
        assertThrows(IllegalStateException.class, dioEncoder::canbus);
    }

    @Test
    void encoderPositionConversionRoundTripsConfiguredMechanismUnits() {
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 1)
                .inverted()
                .offset(0.25)
                .gearRatio(0.5)
                .conversion(360.0);

        double rawRotations = encoder.rotationsFromPosition(-90.0);

        assertEquals(0.25, rawRotations, 1.0e-9);
        assertEquals(-90.0, encoder.positionFromRotations(rawRotations), 1.0e-9);
        assertEquals(-180.0, encoder.velocityFromRotationsPerSecond(1.0), 1.0e-9);
    }

    @Test
    void unifiedHardwareBusValidatesPhysicalInterfacesAtDeclarationTime() {
        HardwareBus rio = HardwareBus.rio();
        HardwareBus canivore = HardwareBus.can("canivore");

        EncoderDevice throughBore = rio.dio(2).encoder(EncoderKinds.REV_THROUGH_BORE);
        ImuDevice navx = rio.spi(SpiPort.MXP).imu(ImuKinds.NAVX);
        DigitalInputDevice limit = rio.dio(3).digitalInput("arm home");

        assertInstanceOf(HardwareAddress.Dio.class, throughBore.connection());
        assertInstanceOf(HardwareAddress.Spi.class, navx.connection());
        assertEquals("encoder:rev_through_bore:rio:2:dio", HardwareIdentity.encoder(throughBore).key());
        assertEquals("imu:studica_navx:rio:4:spi:mxp", HardwareIdentity.imu(navx).key());
        assertEquals("arm_home", limit.defaultName());
        assertEquals(true, rio.supports(HardwareInterface.DIO));
        assertEquals(false, canivore.supports(HardwareInterface.DIO));
        assertThrows(IllegalArgumentException.class,
                () -> canivore.dio(0));
        assertThrows(IllegalArgumentException.class,
                () -> canivore.i2c(I2cPort.MXP));
        assertThrows(IllegalArgumentException.class,
                () -> rio.dio(0).motor(MotorKinds.KRAKEN_X60));
    }

    @Test
    void digitalInputBindingAppliesInversion() {
        boolean[] raw = {true};
        DigitalInputDevice direct = DigitalInputDevice.rio(0).bind(() -> raw[0]);
        DigitalInputDevice inverted = DigitalInputDevice.rio(1).inverted().bind(() -> raw[0]);
        DigitalInputDevice runtimeBound = DigitalInputDevice.rio(2).inverted();
        DigitalInputDevice.bindRuntime(runtimeBound, () -> raw[0]);

        assertEquals(true, direct.active());
        assertEquals(false, inverted.active());
        assertEquals(false, runtimeBound.active());

        raw[0] = false;
        assertEquals(false, direct.active());
        assertEquals(true, inverted.active());
        assertEquals(true, runtimeBound.active());
        assertThrows(IllegalStateException.class, () -> DigitalInputDevice.rio(3).active());
    }

    @Test
    void digitalInputCleanupDoesNotClearNewlySampledEdges() {
        boolean[] raw = {false};
        DigitalInputDevice input = DigitalInputDevice.rio(8).bind(() -> raw[0]);

        input.sample();
        raw[0] = true;
        input.sample();
        assertEquals(true, input.risingLatched());

        raw[0] = false;
        input.sample();
        raw[0] = true;
        input.sample();
        input.clearLatchedEdges();

        assertEquals(true, input.risingLatched());
        input.clearLatchedEdges();
        assertEquals(false, input.risingLatched());
    }

    @Test
    void digitalInputReadsOneCachedSampleUntilNextSample() {
        int[] reads = {0};
        boolean[] raw = {false};
        DigitalInputDevice input = DigitalInputDevice.rio(9).bind(() -> {
            reads[0]++;
            return raw[0];
        });

        input.sample();
        raw[0] = true;

        assertEquals(false, input.raw());
        assertEquals(false, input.active());
        assertEquals(1, reads[0]);

        input.sample();
        assertEquals(true, input.active());
        assertEquals(2, reads[0]);
    }

    @Test
    void refreshPublishesOneImmutableHardwareSnapshotPerCycle() {
        FakeMotorBackend backend = new FakeMotorBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(backend));
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 19);
        FakeMotorHandle handle = assertInstanceOf(FakeMotorHandle.class, graph.motor(motor));
        handle.position = 2.5;

        graph.refreshInputs(100L);
        HardwareCycleSnapshot first = graph.cycleSnapshot();
        handle.position = 8.0;

        assertEquals(1, first.sequence());
        assertEquals(100L, first.timestampNanos());
        assertEquals(2.5, first.motor(motor).orElseThrow().positionRotations(), 1.0e-9);
        assertEquals(1, handle.refreshCalls);
        assertEquals(1, handle.positionReadCalls);
        assertSame(first, graph.cycleSnapshot());

        graph.refreshInputs(200L);

        assertEquals(2, graph.cycleSnapshot().sequence());
        assertEquals(8.0, graph.cycleSnapshot().motor(motor).orElseThrow().positionRotations(), 1.0e-9);
        assertEquals(2, handle.refreshCalls);
        assertEquals(2, handle.positionReadCalls);
        assertEquals(2.5, first.motor(motor).orElseThrow().positionRotations(), 1.0e-9);
    }

    @Test
    void simModelsComposeMotorsEncodersRangesLimitsAndDependencies() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 11);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 12);
        DigitalInputDevice limit = DigitalInputDevice.rio(4);
        Range range = Range.of(-2.0, 2.0);
        SimModel model = SimModel.arm(motor)
                .encoder(encoder)
                .gearRatio(GearRatio.reduction(12.0, 48.0))
                .range(range)
                .limit(limit, 1.5, 0.1);

        assertEquals(List.of(motor), model.motors());
        assertEquals(List.of(encoder), model.encoders());
        assertEquals(true, model.simulatesGravity());
        assertEquals(range, model.range());
        assertEquals(1, model.limits().size());
        SimLimit simLimit = model.limits().get(0);
        assertEquals(limit, simLimit.sensor());
        assertEquals(1.5, simLimit.position(), 1.0e-9);
        assertEquals(0.1, simLimit.tolerance(), 1.0e-9);
    }

    @Test
    void missingBackendUsesSafeHandleByDefaultButPanicPropagates() {
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(List.of(), List.of(), List.of()));
        MotorDevice tolerant = MotorDevice.of(MotorKinds.KRAKEN_X60, 31);

        MotorHandle handle = graph.motor(tolerant);
        handle.setVoltage(12.0);

        assertEquals(0.0, handle.appliedVoltage());
        assertEquals(1, graph.bindingFailures().size());
        assertSame(tolerant, graph.bindingFailures().get(0).declaration());

        MotorDevice panic = MotorDevice.of(MotorKinds.KRAKEN_X60, 32)
                .failurePolicy(FailurePolicy.PANIC);
        assertThrows(IllegalStateException.class, () -> graph.motor(panic));
    }

    private static final class FakeMotorBackend implements MotorBackend {
        private int created;
        private int failuresRemaining;
        private FakeMotorHandle last;

        @Override
        public boolean supports(MotorKind kind) {
            return kind == MotorKinds.KRAKEN_X60;
        }

        @Override
        public MotorHandle create(MotorDevice device) {
            created++;
            if (failuresRemaining-- > 0) {
                throw new IllegalStateException("transient motor creation failure");
            }
            last = new FakeMotorHandle(device);
            return last;
        }

        @Override
        public boolean supportsHardwareFollowing(MotorDevice follower, MotorDevice leader) {
            return follower.canbus().equalsIgnoreCase(leader.canbus());
        }
    }

    private static final class FakeEncoderBackend implements EncoderBackend {
        private int created;
        private int failuresRemaining;
        private FakeEncoderHandle last;

        @Override
        public boolean supports(EncoderKind kind) {
            return kind == EncoderKinds.CANCODER;
        }

        @Override
        public EncoderHandle create(EncoderDevice device) {
            created++;
            if (failuresRemaining-- > 0) {
                throw new IllegalStateException("transient encoder creation failure");
            }
            last = new FakeEncoderHandle(device);
            return last;
        }
    }

    private static final class FakeImuBackend implements ImuBackend {
        private int created;
        private int failuresRemaining;
        private FakeImuHandle last;

        @Override
        public boolean supports(ImuKind kind) {
            return kind == ImuKinds.PIGEON_2;
        }

        @Override
        public ImuHandle create(ImuDevice device) {
            created++;
            if (failuresRemaining-- > 0) {
                throw new IllegalStateException("transient IMU creation failure");
            }
            last = new FakeImuHandle(device);
            return last;
        }
    }

    private static final class FakeMotorHandle implements MotorHandle, AutoCloseable {
        private final MotorDevice device;
        private double position;
        private double velocity;
        private double absolutePosition;
        private double absoluteVelocity;
        private int activateCalls;
        private int closeCalls;
        private int refreshCalls;
        private int positionReadCalls;
        private boolean failRefresh;
        private MotorHandle followLeader;
        private boolean followInverted;
        private int enableIntegratedEncoderCalls;
        private int enableAbsoluteEncoderCalls;
        private double percent;
        private double voltage;
        private double positionTarget;
        private double velocityTarget;
        private int stopCalls;
        private CountDownLatch refreshEntered;
        private CountDownLatch allowRefresh;
        private CountDownLatch positionReadEntered;
        private CountDownLatch allowPositionRead;

        private FakeMotorHandle(MotorDevice device) {
            this.device = device;
        }

        @Override
        public MotorDevice device() {
            return device;
        }

        @Override
        public void activate() {
            activateCalls++;
        }

        @Override
        public void refreshInputs() {
            refreshCalls++;
            signalAndAwait(refreshEntered, allowRefresh);
            if (failRefresh) {
                throw new IllegalStateException("refresh failed");
            }
        }

        @Override
        public void follow(MotorHandle leader, boolean inverted) {
            followLeader = leader;
            followInverted = inverted;
        }

        @Override public void setPercentOutput(double value) { percent = value; }
        @Override public void setVoltage(double value) { voltage = value; }
        @Override public void setPositionTargetRotations(double value) { positionTarget = value; }
        @Override public void setVelocityTargetRotationsPerSecond(double value) { velocityTarget = value; }
        @Override public void stop() { stopCalls++; }

        @Override
        public double integratedPositionRotations() {
            positionReadCalls++;
            signalAndAwait(positionReadEntered, allowPositionRead);
            return position;
        }

        private void blockRefresh() {
            refreshEntered = new CountDownLatch(1);
            allowRefresh = new CountDownLatch(1);
        }

        private void blockPositionRead() {
            positionReadEntered = new CountDownLatch(1);
            allowPositionRead = new CountDownLatch(1);
        }

        private void releaseBlocks() {
            if (allowRefresh != null) allowRefresh.countDown();
            if (allowPositionRead != null) allowPositionRead.countDown();
        }

        private static void signalAndAwait(CountDownLatch entered, CountDownLatch release) {
            if (entered == null || release == null) return;
            entered.countDown();
            try {
                if (!release.await(5, TimeUnit.SECONDS)) {
                    throw new IllegalStateException("Timed out waiting for blocked test operation to resume.");
                }
            } catch (InterruptedException exception) {
                Thread.currentThread().interrupt();
                throw new IllegalStateException("Interrupted while blocking test operation.", exception);
            }
        }

        @Override
        public double integratedVelocityRotationsPerSecond() {
            return velocity;
        }

        @Override
        public double absolutePositionRotations() {
            return absolutePosition;
        }

        @Override
        public double absoluteVelocityRotationsPerSecond() {
            return absoluteVelocity;
        }

        @Override
        public void enableIntegratedEncoder() {
            enableIntegratedEncoderCalls++;
        }

        @Override
        public void enableAbsoluteEncoder() {
            enableAbsoluteEncoderCalls++;
        }

        @Override
        public void close() {
            closeCalls++;
        }
    }

    private static final class FakeEncoderHandle implements EncoderHandle, AutoCloseable {
        private final EncoderDevice device;
        private int activateCalls;
        private int closeCalls;
        private double position;
        private double absolutePosition;
        private double velocity;

        private FakeEncoderHandle(EncoderDevice device) {
            this.device = device;
        }

        @Override
        public EncoderDevice device() {
            return device;
        }

        @Override
        public void activate() {
            activateCalls++;
        }

        @Override
        public double positionRotations() {
            return position;
        }

        @Override
        public double absolutePositionRotations() {
            return absolutePosition;
        }

        @Override
        public double velocityRotationsPerSecond() {
            return velocity;
        }

        @Override
        public void close() {
            closeCalls++;
        }
    }

    private static final class FakeImuHandle implements ImuHandle, AutoCloseable {
        private final ImuDevice device;
        private int activateCalls;
        private int closeCalls;
        private double yaw;

        private FakeImuHandle(ImuDevice device) {
            this.device = device;
        }

        @Override
        public ImuDevice device() {
            return device;
        }

        @Override
        public void activate() {
            activateCalls++;
        }

        @Override
        public double yawDegrees() {
            return yaw;
        }

        @Override
        public void close() {
            closeCalls++;
        }
    }
}
