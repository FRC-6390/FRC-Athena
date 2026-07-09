package ca.frc6390.athena.vendor.rev;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import org.junit.jupiter.api.Test;

class RevBackendTest {
    @Test
    void motorBackendSupportsBuiltInAndEquivalentKeys() {
        RevMotorBackend backend = new RevMotorBackend();

        assertTrue(backend.supports(MotorKinds.SPARK_MAX_BRUSHLESS));
        assertTrue(backend.supports((MotorKind) () -> "rev:spark-flex-brushed"));
        assertFalse(backend.supports((MotorKind) () -> "ctre:kraken-x60"));
    }

    @Test
    void encoderBackendSupportsThroughBoreOnly() {
        RevEncoderBackend backend = new RevEncoderBackend();

        assertTrue(backend.supports(EncoderKinds.REV_THROUGH_BORE));
        assertFalse(backend.supports((EncoderKind) () -> "ctre:cancoder"));
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
                EncoderDevice.of(EncoderKinds.REV_THROUGH_BORE, 4),
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
                MotorDevice.of(MotorKinds.SPARK_MAX_BRUSHLESS, 1).neutralMode(MotorNeutralMode.BRAKE),
                options,
                controller);

        assertEquals(0, controller.configureCalls);

        handle.activate();
        handle.activate();

        assertEquals(1, controller.configureCalls);
        assertEquals(options, controller.options);
        assertEquals(MotorNeutralMode.BRAKE, controller.device.neutralMode());
    }

    private static final class RecordingSparkController implements RevMotorHandle.SparkController {
        private int configureCalls;
        private MotorDevice device;
        private RevMotorOptions options;

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
        public void stop() {}

        @Override
        public double positionRotations() {
            return 0.0;
        }

        @Override
        public double velocityRotationsPerSecond() {
            return 0.0;
        }

        @Override
        public double absolutePositionRotations() {
            return 0.0;
        }

        @Override
        public double absoluteVelocityRotationsPerSecond() {
            return 0.0;
        }
    }
}
