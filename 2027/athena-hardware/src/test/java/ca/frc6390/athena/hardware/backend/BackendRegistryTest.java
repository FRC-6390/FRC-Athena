package ca.frc6390.athena.hardware.backend;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.api.hardware.AthenaImu;
import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.capability.CapabilitySet;
import ca.frc6390.athena.hardware.capability.MotorCapability;
import ca.frc6390.athena.hardware.encoder.EncoderSpec;
import ca.frc6390.athena.hardware.imu.ImuSpec;
import ca.frc6390.athena.hardware.spec.MotorSpec;
import ca.frc6390.athena.hardware.spec.NeutralMode;

class BackendRegistryTest {
    @Test
    void findsSupportingMotorBackend() {
        MotorBackend backend = new SimBackend();
        BackendRegistry registry = BackendRegistry.of(backend);

        assertSame(backend, registry.motorBackendFor(AthenaMotor.SIM).orElseThrow());
        assertFalse(registry.motorBackendFor(AthenaMotor.TALON_FX).isPresent());
    }

    @Test
    void findsSupportingImuBackend() {
        ImuBackend backend = new SimImuBackend();
        BackendRegistry registry = BackendRegistry.of(List.of(), List.of(), List.of(backend));

        assertSame(backend, registry.imuBackendFor(AthenaImu.SIM).orElseThrow());
        assertFalse(registry.imuBackendFor(AthenaImu.PIGEON_2).isPresent());
    }

    @Test
    void findsSupportingEncoderBackend() {
        EncoderBackend backend = new SimEncoderBackend();
        BackendRegistry registry = BackendRegistry.of(List.of(), List.of(backend), List.of());

        assertSame(backend, registry.encoderBackendFor(AthenaEncoder.SIM).orElseThrow());
        assertFalse(registry.encoderBackendFor(AthenaEncoder.CANCODER).isPresent());
    }

    @Test
    void globalRegistryCanBeReplacedAndRestored() {
        BackendRegistry original = BackendRegistry.global();
        BackendRegistry replacement = BackendRegistry.of(new SimBackend());

        try {
            BackendRegistry.setGlobal(replacement);
            assertTrue(BackendRegistry.global().motorBackendFor(AthenaMotor.SIM).isPresent());
        } finally {
            BackendRegistry.setGlobal(original);
        }
    }

    @Test
    void backendCreatesDeviceFromSpec() {
        MotorSpec spec = new MotorSpec(
                "intake",
                "roller",
                AthenaMotor.SIM,
                1,
                "rio",
                NeutralMode.BRAKE,
                35,
                false);

        MotorDevice device = new SimBackend().create(spec);

        assertEquals(spec, device.spec());
    }

    private static final class SimBackend implements MotorBackend {
        @Override
        public boolean supports(MotorKind kind) {
            return kind == AthenaMotor.SIM;
        }

        @Override
        public CapabilitySet capabilities(MotorKind kind) {
            return CapabilitySet.of(MotorCapability.PERCENT_OUTPUT);
        }

        @Override
        public MotorDevice create(MotorSpec spec) {
            return () -> spec;
        }
    }

    private static final class SimImuBackend implements ImuBackend {
        @Override
        public boolean supports(ImuKind kind) {
            return kind == AthenaImu.SIM;
        }

        @Override
        public ImuDevice create(ImuSpec spec) {
            return new ImuDevice() {
                @Override
                public ImuSpec spec() {
                    return spec;
                }

                @Override
                public double yawDegrees() {
                    return 0.0;
                }
            };
        }
    }

    private static final class SimEncoderBackend implements EncoderBackend {
        @Override
        public boolean supports(EncoderKind kind) {
            return kind == AthenaEncoder.SIM;
        }

        @Override
        public EncoderDevice create(EncoderSpec spec) {
            return new EncoderDevice() {
                @Override
                public EncoderSpec spec() {
                    return spec;
                }
            };
        }
    }
}
