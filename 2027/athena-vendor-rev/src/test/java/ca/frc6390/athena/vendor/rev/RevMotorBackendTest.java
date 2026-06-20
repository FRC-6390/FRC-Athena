package ca.frc6390.athena.vendor.rev;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.hardware.capability.MotorCapability;
import ca.frc6390.athena.hardware.config.MotorConfig;

class RevMotorBackendTest {
    private final RevMotorBackend backend = new RevMotorBackend();

    @Test
    void supportsSparkMotorKindsOnly() {
        assertTrue(backend.supports(AthenaMotor.SPARK_MAX_BRUSHLESS));
        assertTrue(backend.supports(AthenaMotor.SPARK_FLEX_BRUSHED));
        assertFalse(backend.supports(AthenaMotor.TALON_FX));
    }

    @Test
    void reportsClosedLoopAndIntegratedEncoderCapabilities() {
        var capabilities = backend.capabilities(AthenaMotor.SPARK_FLEX_BRUSHLESS);

        assertTrue(capabilities.contains(MotorCapability.VELOCITY_CLOSED_LOOP));
        assertTrue(capabilities.contains(MotorCapability.POSITION_CLOSED_LOOP));
        assertTrue(capabilities.contains(MotorCapability.INTEGRATED_ENCODER));
        assertTrue(capabilities.contains(MotorCapability.ABSOLUTE_ENCODER));
    }

    @Test
    void createsDeviceWithTypedRevOptions() {
        var spec = MotorConfig.create()
                .hardware(AthenaMotor.SPARK_FLEX_BRUSHLESS, 21)
                .vendor(RevMotorOptions.class, rev -> rev
                        .smartCurrentLimit(50)
                        .openLoopRampSeconds(0.2))
                .toSpec("arm", "pivot");

        var device = new RevMotorDevice(spec, spec.vendorOptions().find(RevMotorOptions.class).orElseThrow(), new RecordingSpark());

        assertEquals(spec, device.spec());
        assertEquals(50, device.options().smartCurrentLimitAmps());
        assertEquals(0.2, device.options().openLoopRampSeconds(), 1.0e-9);
    }

    @Test
    void deviceWritesPercentVoltageAndStopToRevController() {
        var spec = MotorConfig.create()
                .hardware(AthenaMotor.SPARK_MAX_BRUSHLESS, 21)
                .toSpec("drive", "right");
        var controller = new RecordingSpark();
        var device = new RevMotorDevice(spec, new RevMotorOptions(), controller);

        device.setPercentOutput(-2.0);
        device.setVoltage(6.0);
        device.setPositionTargetRotations(0.75);
        device.setVelocityTargetRotationsPerSecond(12.5);
        device.stop();
        controller.position = 9.5;
        controller.velocity = 20.0;
        controller.absolutePosition = 0.25;
        controller.absoluteVelocity = 1.5;

        assertEquals(-1.0, controller.percent, 1.0e-9);
        assertEquals(6.0, controller.volts, 1.0e-9);
        assertEquals(0.75, controller.positionTarget, 1.0e-9);
        assertEquals(12.5, controller.velocityTarget, 1.0e-9);
        assertEquals(true, controller.stopped);
        assertEquals(9.5, device.integratedPositionRotations(), 1.0e-9);
        assertEquals(20.0, device.integratedVelocityRotationsPerSecond(), 1.0e-9);
        assertEquals(0.25, device.absolutePositionRotations(), 1.0e-9);
        assertEquals(1.5, device.absoluteVelocityRotationsPerSecond(), 1.0e-9);
    }

    private static final class RecordingSpark implements RevMotorDevice.SparkController {
        private double percent;
        private double volts;
        private double position;
        private double velocity;
        private double positionTarget;
        private double velocityTarget;
        private double absolutePosition;
        private double absoluteVelocity;
        private boolean stopped;

        @Override
        public void configure(ca.frc6390.athena.hardware.spec.MotorSpec spec, RevMotorOptions options) {
        }

        @Override
        public void setPercent(double percent) {
            this.percent = percent;
        }

        @Override
        public void setVoltage(double volts) {
            this.volts = volts;
        }

        @Override
        public void setPositionTarget(double rotations) {
            positionTarget = rotations;
        }

        @Override
        public void setVelocityTarget(double rotationsPerSecond) {
            velocityTarget = rotationsPerSecond;
        }

        @Override
        public void stop() {
            stopped = true;
        }

        @Override
        public double positionRotations() {
            return position;
        }

        @Override
        public double velocityRotationsPerSecond() {
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
    }
}
