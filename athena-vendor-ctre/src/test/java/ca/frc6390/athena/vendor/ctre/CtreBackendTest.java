package ca.frc6390.athena.vendor.ctre;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import org.junit.jupiter.api.Test;

class CtreBackendTest {
    @Test
    void motorBackendSupportsBuiltInAndEquivalentKeys() {
        CtreMotorBackend backend = new CtreMotorBackend();

        assertTrue(backend.supports(MotorKinds.TALON_FX));
        assertTrue(backend.supports(() -> "ctre:kraken-x60"));
        assertFalse(backend.supports((MotorKind) () -> "rev:spark-max-brushless"));
    }

    @Test
    void encoderBackendSupportsCanCoderOnly() {
        CtreEncoderBackend backend = new CtreEncoderBackend();

        assertTrue(backend.supports(EncoderKinds.CANCODER));
        assertTrue(backend.supports((EncoderKind) () -> "ctre:cancoder"));
        assertFalse(backend.supports((EncoderKind) () -> "rev:through-bore"));
    }

    @Test
    void imuBackendSupportsPigeon2Only() {
        CtreImuBackend backend = new CtreImuBackend();

        assertTrue(backend.supports(ImuKinds.PIGEON_2));
        assertTrue(backend.supports((ImuKind) () -> "ctre:pigeon-2"));
        assertFalse(backend.supports((ImuKind) () -> "studica:navx"));
    }

    @Test
    void motorOptionsClampNegativeCurrentLimits() {
        CtreMotorOptions options = new CtreMotorOptions()
                .supplyCurrentLimit(-1)
                .statorCurrentLimit(-2)
                .torqueCurrentLimit(35);

        assertEquals(0, options.supplyCurrentLimitAmps());
        assertEquals(0, options.statorCurrentLimitAmps());
        assertEquals(35, options.torqueCurrentLimitAmps());
    }

    @Test
    void motorConfigurationRunsDuringActivationOnlyOnce() {
        RecordingTalonController controller = new RecordingTalonController();
        CtreMotorHandle handle = new CtreMotorHandle(
                MotorDevice.of(MotorKinds.TALON_FX, 1).neutralMode(MotorNeutralMode.BRAKE),
                new CtreMotorOptions(),
                controller);

        assertEquals(0, controller.neutralModeCalls);

        handle.activate();
        handle.activate();

        assertEquals(1, controller.neutralModeCalls);
        assertTrue(controller.brake);
    }

    @Test
    void motorInputsAreCachedUntilNextRefresh() {
        RecordingTalonController controller = new RecordingTalonController();
        CtreMotorHandle handle = new CtreMotorHandle(MotorDevice.of(MotorKinds.TALON_FX, 2), null, controller);

        handle.refreshInputs();

        assertEquals(10.0, handle.integratedPositionRotations(), 1.0e-9);
        assertEquals(5.0, handle.integratedVelocityRotationsPerSecond(), 1.0e-9);
        assertEquals(10.0, handle.integratedPositionRotations(), 1.0e-9);
        assertEquals(1, controller.positionCalls);
        assertEquals(1, controller.velocityCalls);
    }

    @Test
    void encoderInputsAreCachedUntilNextRefresh() {
        RecordingCancoderController controller = new RecordingCancoderController();
        CtreEncoderHandle handle = new CtreEncoderHandle(EncoderDevice.of(EncoderKinds.CANCODER, 3), controller);

        handle.refreshInputs();

        assertEquals(1.5, handle.positionRotations(), 1.0e-9);
        assertEquals(0.25, handle.absolutePositionRotations(), 1.0e-9);
        assertEquals(2.5, handle.velocityRotationsPerSecond(), 1.0e-9);
        assertEquals(1.5, handle.positionRotations(), 1.0e-9);
        assertEquals(1, controller.positionCalls);
        assertEquals(1, controller.absolutePositionCalls);
        assertEquals(1, controller.velocityCalls);
    }

    @Test
    void pigeonInputsAreCachedUntilNextRefresh() {
        RecordingPigeonController controller = new RecordingPigeonController();
        CtrePigeon2Handle handle = new CtrePigeon2Handle(ImuDevice.of(ImuKinds.PIGEON_2, 4), controller);

        handle.refreshInputs();

        assertEquals(12.0, handle.yawDegrees(), 1.0e-9);
        assertEquals(3.0, handle.pitchDegrees(), 1.0e-9);
        assertEquals(4.0, handle.rollDegrees(), 1.0e-9);
        assertEquals(12.0, handle.yawDegrees(), 1.0e-9);
        assertEquals(1, controller.yawCalls);
        assertEquals(1, controller.pitchCalls);
        assertEquals(1, controller.rollCalls);
    }

    private static final class RecordingTalonController implements CtreMotorHandle.TalonController {
        private int neutralModeCalls;
        private boolean brake;

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
        public void setNeutralMode(boolean brake) {
            neutralModeCalls++;
            this.brake = brake;
        }

        @Override
        public double positionRotations() {
            positionCalls++;
            return 10.0;
        }

        @Override
        public double velocityRotationsPerSecond() {
            velocityCalls++;
            return 5.0;
        }

        private int positionCalls;
        private int velocityCalls;
    }

    private static final class RecordingCancoderController implements CtreEncoderHandle.CANCoderController {
        private int positionCalls;
        private int absolutePositionCalls;
        private int velocityCalls;

        @Override
        public double positionRotations() {
            positionCalls++;
            return 1.5;
        }

        @Override
        public double absolutePositionRotations() {
            absolutePositionCalls++;
            return 0.25;
        }

        @Override
        public double velocityRotationsPerSecond() {
            velocityCalls++;
            return 2.5;
        }
    }

    private static final class RecordingPigeonController implements CtrePigeon2Handle.Pigeon2Controller {
        private int yawCalls;
        private int pitchCalls;
        private int rollCalls;

        @Override
        public double yawDegrees() {
            yawCalls++;
            return 12.0;
        }

        @Override
        public double pitchDegrees() {
            pitchCalls++;
            return 3.0;
        }

        @Override
        public double rollDegrees() {
            rollCalls++;
            return 4.0;
        }

        @Override
        public void setYawDegrees(double yawDegrees) {}

        @Override
        public void reset() {}

        @Override
        public void close() {}
    }
}
