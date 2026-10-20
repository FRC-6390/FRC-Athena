package ca.frc6390.athena.hardware.runtime;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

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
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import org.junit.jupiter.api.Test;

/** Checks the per-cycle snapshot contents, which the graph builds from reused entry buffers. */
class HardwareCycleSnapshotCaptureTest {
    @Test
    void snapshotHoldsEveryDeviceWithFreshValuesEachCycle() {
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(
                List.of(new MotorBackendStub()), List.of(new EncoderBackendStub()), List.of(new ImuBackendStub())));
        MotorDevice first = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        MotorDevice second = MotorDevice.of(MotorKinds.NEO, 2);
        EncoderDevice encoder = EncoderDevice.of(EncoderKinds.CANCODER, 3);
        ImuDevice imu = ImuDevice.of(ImuKinds.PIGEON_2, 4);
        MotorStub firstHandle = (MotorStub) graph.motor(first);
        MotorStub secondHandle = (MotorStub) graph.motor(second);
        EncoderStub encoderHandle = (EncoderStub) graph.encoder(encoder);
        ImuStub imuHandle = (ImuStub) graph.imu(imu);

        firstHandle.position = 1.5;
        secondHandle.position = -2.0;
        encoderHandle.position = 0.25;
        imuHandle.yaw = 90.0;
        graph.refreshInputs(1_000L);
        HardwareCycleSnapshot earlier = graph.cycleSnapshot();

        firstHandle.position = 3.0;
        encoderHandle.position = 0.75;
        imuHandle.yaw = 45.0;
        graph.refreshInputs(2_000L);
        HardwareCycleSnapshot later = graph.cycleSnapshot();

        assertNotSame(earlier, later);
        assertEquals(1.5, earlier.motor(first).orElseThrow().positionRotations());
        assertEquals(-2.0, earlier.motor(second).orElseThrow().positionRotations());
        assertEquals(0.25, earlier.encoder(encoder).orElseThrow().positionRotations());
        assertEquals(90.0, earlier.imu(imu).orElseThrow().yawDegrees());
        assertEquals(3.0, later.motor(first).orElseThrow().positionRotations());
        assertEquals(-2.0, later.motor(second).orElseThrow().positionRotations());
        assertEquals(0.75, later.encoder(encoder).orElseThrow().positionRotations());
        assertEquals(45.0, later.imu(imu).orElseThrow().yawDegrees());
        assertEquals(first.defaultName(), later.motor(first).orElseThrow().name());
        assertEquals(encoder.defaultName(), later.encoder(encoder).orElseThrow().name());
        assertEquals(imu.defaultName(), later.imu(imu).orElseThrow().name());
        assertTrue(later.motor(first).orElseThrow().connected());

        Map<HardwareIdentity, HardwareCycleSnapshot.MotorInput> expected = new LinkedHashMap<>();
        expected.put(HardwareIdentity.motor(first), later.motor(first).orElseThrow());
        expected.put(HardwareIdentity.motor(second), later.motor(second).orElseThrow());
        assertEquals(expected, later.motors());
        assertEquals(1, later.encoders().size());
        assertEquals(1, later.imus().size());
        assertThrows(UnsupportedOperationException.class, () -> later.motors().clear());
        graph.close();
    }

    @Test
    void emptyGraphPublishesEmptySnapshotMaps() {
        HardwareGraph graph = HardwareGraph.using(BackendRegistry.of(new MotorBackendStub()));
        graph.refreshInputs(1_000L);

        HardwareCycleSnapshot snapshot = graph.cycleSnapshot();
        assertTrue(snapshot.captured());
        assertEquals(Map.of(), snapshot.motors());
        assertEquals(Map.of(), snapshot.encoders());
        assertEquals(Map.of(), snapshot.imus());
        graph.close();
    }

    private static final class MotorBackendStub implements MotorBackend {
        @Override
        public boolean supports(MotorKind kind) {
            return true;
        }

        @Override
        public MotorHandle create(MotorDevice device) {
            return new MotorStub(device);
        }
    }

    private static final class EncoderBackendStub implements EncoderBackend {
        @Override
        public boolean supports(EncoderKind kind) {
            return true;
        }

        @Override
        public EncoderHandle create(EncoderDevice device) {
            return new EncoderStub(device);
        }
    }

    private static final class ImuBackendStub implements ImuBackend {
        @Override
        public boolean supports(ImuKind kind) {
            return true;
        }

        @Override
        public ImuHandle create(ImuDevice device) {
            return new ImuStub(device);
        }
    }

    private static final class MotorStub implements MotorHandle {
        private final MotorDevice device;
        private double position;

        private MotorStub(MotorDevice device) {
            this.device = device;
        }

        @Override public MotorDevice device() { return device; }
        @Override public void setPercentOutput(double percent) { }
        @Override public void setVoltage(double volts) { }
        @Override public double appliedVoltage() { return 0.0; }
        @Override public double supplyCurrentAmps() { return 0.0; }
        @Override public double statorCurrentAmps() { return 0.0; }
        @Override public double integratedPositionRotations() { return position; }
        @Override public double integratedVelocityRotationsPerSecond() { return 0.0; }
        @Override public double absolutePositionRotations() { return 0.0; }
        @Override public double absoluteVelocityRotationsPerSecond() { return 0.0; }
        @Override public void setIntegratedPositionRotations(double rotations) { position = rotations; }
        @Override public boolean supportsIntegratedPositionSetting() { return true; }
    }

    private static final class EncoderStub implements EncoderHandle {
        private final EncoderDevice device;
        private double position;

        private EncoderStub(EncoderDevice device) {
            this.device = device;
        }

        @Override public EncoderDevice device() { return device; }
        @Override public double positionRotations() { return position; }
        @Override public double absolutePositionRotations() { return position; }
        @Override public double velocityRotationsPerSecond() { return 0.0; }
        @Override public void setPositionRotations(double rotations) { position = rotations; }
        @Override public boolean supportsPositionSetting() { return true; }
    }

    private static final class ImuStub implements ImuHandle {
        private final ImuDevice device;
        private double yaw;

        private ImuStub(ImuDevice device) {
            this.device = device;
        }

        @Override public ImuDevice device() { return device; }
        @Override public double yawDegrees() { return yaw; }
        @Override public double pitchDegrees() { return 0.0; }
        @Override public double rollDegrees() { return 0.0; }
        @Override public double yawRateDegreesPerSecond() { return 0.0; }
        @Override public double linearAccelerationXG() { return 0.0; }
        @Override public double linearAccelerationYG() { return 0.0; }
        @Override public double linearAccelerationZG() { return 0.0; }
        @Override public void setYawDegrees(double yawDegrees) { yaw = yawDegrees; }
    }
}
