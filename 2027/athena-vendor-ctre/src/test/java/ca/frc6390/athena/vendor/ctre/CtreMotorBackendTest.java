package ca.frc6390.athena.vendor.ctre;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.hardware.capability.MotorCapability;
import ca.frc6390.athena.hardware.config.MotorConfig;
import ca.frc6390.athena.hardware.spec.NeutralMode;

class CtreMotorBackendTest {
    private final CtreMotorBackend backend = new CtreMotorBackend();

    @Test
    void supportsPhoenix6TalonFxFamily() {
        assertTrue(backend.supports(AthenaMotor.TALON_FX));
        assertTrue(backend.supports(AthenaMotor.KRAKEN_X60));
        assertTrue(backend.supports(AthenaMotor.KRAKEN_X44));
        assertFalse(backend.supports(AthenaMotor.SPARK_MAX_BRUSHLESS));
    }

    @Test
    void reportsClosedLoopAndIntegratedEncoderCapabilities() {
        var capabilities = backend.capabilities(AthenaMotor.TALON_FX);

        assertTrue(capabilities.contains(MotorCapability.VELOCITY_CLOSED_LOOP));
        assertTrue(capabilities.contains(MotorCapability.POSITION_CLOSED_LOOP));
        assertTrue(capabilities.contains(MotorCapability.INTEGRATED_ENCODER));
    }

    @Test
    void createsDeviceWithTypedCtreOptions() {
        var spec = MotorConfig.create()
                .hardware(AthenaMotor.TALON_FX, 12)
                .canbus("canivore")
                .brake()
                .vendor(CtreMotorOptions.class, ctre -> ctre
                        .statorCurrentLimit(80)
                        .supplyCurrentLimit(50))
                .toSpec("shooter", "leader");

        var device = new CtreMotorDevice(spec, spec.vendorOptions().find(CtreMotorOptions.class).orElseThrow(), new RecordingTalon());

        assertEquals(spec, device.spec());
        assertEquals(NeutralMode.BRAKE, device.spec().neutralMode());
        assertEquals(80, device.options().statorCurrentLimitAmps());
        assertEquals(50, device.options().supplyCurrentLimitAmps());
    }

    @Test
    void deviceWritesPercentVoltageAndStopToPhoenixController() {
        var spec = MotorConfig.create()
                .hardware(AthenaMotor.TALON_FX, 12)
                .toSpec("drive", "left");
        var controller = new RecordingTalon();
        var device = new CtreMotorDevice(spec, new CtreMotorOptions(), controller);

        device.setPercentOutput(2.0);
        device.setVoltage(7.5);
        device.setPositionTargetRotations(3.25);
        device.setVelocityTargetRotationsPerSecond(18.0);
        device.stop();
        controller.position = 18.0;
        controller.velocity = 42.0;

        assertEquals(1.0, controller.percent, 1.0e-9);
        assertEquals(7.5, controller.volts, 1.0e-9);
        assertEquals(3.25, controller.positionTarget, 1.0e-9);
        assertEquals(18.0, controller.velocityTarget, 1.0e-9);
        assertEquals(true, controller.stopped);
        assertEquals(18.0, device.integratedPositionRotations(), 1.0e-9);
        assertEquals(42.0, device.integratedVelocityRotationsPerSecond(), 1.0e-9);
    }

    private static final class RecordingTalon implements CtreMotorDevice.TalonController {
        private double percent;
        private double volts;
        private double position;
        private double velocity;
        private double positionTarget;
        private double velocityTarget;
        private boolean stopped;

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
        public void setNeutralMode(boolean brake) {
        }

        @Override
        public double positionRotations() {
            return position;
        }

        @Override
        public double velocityRotationsPerSecond() {
            return velocity;
        }
    }
}
