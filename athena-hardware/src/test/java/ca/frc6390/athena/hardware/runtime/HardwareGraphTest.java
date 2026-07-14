package ca.frc6390.athena.hardware.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;

import java.util.List;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.api.hardware.MotorControllerKinds;
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
    }

    @Test
    void missingBackendsAndDefaultHandleMethodsFailFast() {
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(List.of(), List.of(), List.of()));

        assertThrows(IllegalStateException.class, () -> graph.motor(MotorDevice.of(MotorKinds.KRAKEN_X60, 1)));
        assertThrows(IllegalStateException.class, () -> graph.encoder(EncoderDevice.of(EncoderKinds.CANCODER, 1)));
        assertThrows(IllegalStateException.class, () -> graph.imu(ImuDevice.of(ImuKinds.PIGEON_2, 1)));

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

    private static final class FakeMotorBackend implements MotorBackend {
        private int created;
        private FakeMotorHandle last;

        @Override
        public boolean supports(MotorKind kind) {
            return kind == MotorKinds.KRAKEN_X60;
        }

        @Override
        public MotorHandle create(MotorDevice device) {
            created++;
            last = new FakeMotorHandle(device);
            return last;
        }
    }

    private static final class FakeEncoderBackend implements EncoderBackend {
        private int created;
        private FakeEncoderHandle last;

        @Override
        public boolean supports(EncoderKind kind) {
            return kind == EncoderKinds.CANCODER;
        }

        @Override
        public EncoderHandle create(EncoderDevice device) {
            created++;
            last = new FakeEncoderHandle(device);
            return last;
        }
    }

    private static final class FakeImuBackend implements ImuBackend {
        private int created;
        private FakeImuHandle last;

        @Override
        public boolean supports(ImuKind kind) {
            return kind == ImuKinds.PIGEON_2;
        }

        @Override
        public ImuHandle create(ImuDevice device) {
            created++;
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
        private boolean failRefresh;
        private MotorHandle followLeader;
        private boolean followInverted;

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
            if (failRefresh) {
                throw new IllegalStateException("refresh failed");
            }
        }

        @Override
        public void follow(MotorHandle leader, boolean inverted) {
            followLeader = leader;
            followInverted = inverted;
        }

        @Override
        public double integratedPositionRotations() {
            return position;
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
