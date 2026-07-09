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
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.sim.SimLimit;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.hardware.sim.SimModels;

class HardwareGraphTest {
    @Test
    void hardwareIdentityIncludesCategoryKindBusIdAndIntegratedDetail() {
        MotorDevice motor = MotorDevice.of(MotorKinds.SIM, 7).canbus("CANivore A");
        EncoderDevice encoder = motor.encoder();

        assertEquals("motor:sim_motor:canivore_a:7", HardwareIdentity.motor(motor).key());
        assertEquals("encoder:athena_integrated_motor:canivore_a:7:integrated", HardwareIdentity.encoder(encoder).key());
        assertEquals("encoder:athena_integrated_motor:canivore_a:7:absolute", HardwareIdentity.encoder(motor.absoluteEncoder()).key());
        assertEquals("imu:sim_imu:rio:2", HardwareIdentity.imu(ImuDevice.of(ImuKinds.SIM, 2)).key());
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
        MotorDevice motor = MotorDevice.of(MotorKinds.SIM, 1);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.SIM, 2);
        ImuDevice imu = ImuDevice.of(ImuKinds.SIM, 3);

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
    void integratedEncoderHandleReadsThroughCachedMotorHandle() {
        FakeMotorBackend motorBackend = new FakeMotorBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(List.of(motorBackend), List.of(), List.of()));
        MotorDevice motor = MotorDevice.of(MotorKinds.SIM, 4);
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
    void motorAbsoluteEncoderHandleReadsThroughCachedMotorHandle() {
        FakeMotorBackend motorBackend = new FakeMotorBackend();
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(List.of(motorBackend), List.of(), List.of()));
        MotorDevice motor = MotorDevice.of(MotorKinds.SIM, 5);
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

        assertThrows(IllegalStateException.class, () -> graph.motor(MotorDevice.of(MotorKinds.SIM, 1)));
        assertThrows(IllegalStateException.class, () -> graph.encoder(EncoderDevice.of(EncoderKinds.SIM, 1)));
        assertThrows(IllegalStateException.class, () -> graph.imu(ImuDevice.of(ImuKinds.SIM, 1)));

        MotorHandle defaultMotor = new MotorHandle() {
            @Override
            public MotorDevice device() {
                return MotorDevice.of(MotorKinds.SIM, 9);
            }
        };
        EncoderHandle defaultEncoder = new EncoderHandle() {
            @Override
            public EncoderDevice device() {
                return EncoderDevice.of(EncoderKinds.SIM, 9);
            }
        };

        assertThrows(UnsupportedOperationException.class, () -> defaultMotor.setPercentOutput(0.1));
        assertThrows(UnsupportedOperationException.class, defaultEncoder::positionRotations);
    }

    @Test
    void deviceValidationRejectsInvalidDeclarations() {
        assertThrows(NullPointerException.class, () -> MotorDevice.of(null, 1));
        assertThrows(NullPointerException.class, () -> EncoderDevice.of(null, 1));
        assertThrows(NullPointerException.class, () -> ImuDevice.of(null, 1));
        assertThrows(IllegalArgumentException.class, () -> EncoderDevice.of(EncoderKinds.SIM, 1).gearRatio(0.0));
        assertThrows(IllegalArgumentException.class, () -> EncoderDevice.of(EncoderKinds.SIM, 1).conversion(Double.NaN));
        assertEquals(2, EncoderDevice.of(EncoderKinds.SIM, 2).dioChannel());
        assertThrows(IllegalStateException.class, () -> EncoderDevice.of(EncoderKinds.CANCODER, 3).canbus("canivore").dioChannel());
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
    void simModelsComposeMotorsEncodersRangesLimitsAndDependencies() {
        MotorDevice motor = MotorDevice.of(MotorKinds.SIM, 11);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.SIM, 12);
        DigitalInputDevice limit = DigitalInputDevice.rio(4);
        Range range = Range.of(-2.0, 2.0);
        SimModel model = SimModels.arm(motor)
                .encoder(encoder)
                .gearRatio(GearRatio.reduction(12.0, 48.0))
                .range(range)
                .limit(limit, 1.5, 0.1);

        assertEquals(List.of(motor), model.motors());
        assertEquals(List.of(encoder), model.encoders());
        assertEquals(true, model.simulatesGravity());
        assertEquals(2, model.dependencies().size());
        assertEquals(range, model.dependencies().get(0));
        SimLimit simLimit = assertInstanceOf(SimLimit.class, model.dependencies().get(1));
        assertEquals(limit, simLimit.sensor());
        assertEquals(1.5, simLimit.position(), 1.0e-9);
        assertEquals(0.1, simLimit.tolerance(), 1.0e-9);
    }

    private static final class FakeMotorBackend implements MotorBackend {
        private int created;
        private FakeMotorHandle last;

        @Override
        public boolean supports(MotorKind kind) {
            return kind == MotorKinds.SIM;
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
            return kind == EncoderKinds.SIM;
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
            return kind == ImuKinds.SIM;
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
        public void close() {
            closeCalls++;
        }
    }

    private static final class FakeImuHandle implements ImuHandle, AutoCloseable {
        private final ImuDevice device;
        private int activateCalls;
        private int closeCalls;

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
            return 0.0;
        }

        @Override
        public void close() {
            closeCalls++;
        }
    }
}
