package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import org.junit.jupiter.api.Test;

class FailureRecoveryTest {
    @Test
    void mechanismRemainsSuppressedUntilEveryDeviceFaultRecovers() {
        RecoverableMechanism mechanism = new RecoverableMechanism(false);
        RecordingMotor handle = new RecordingMotor(mechanism.motor);
        MechanismScheduler scheduler = MechanismScheduler.create()
                .motor(mechanism.motor, handle)
                .register(mechanism);
        scheduler.request(mechanism.run);

        scheduler.disableOwner(mechanism.firstEncoder);
        scheduler.disableOwner(mechanism.secondEncoder);
        scheduler.recoverFailure(mechanism.firstEncoder);
        scheduler.robotPeriodic(0.0, 0.02);
        assertEquals(0.0, handle.percent, 1e-9);

        scheduler.recoverFailure(mechanism.secondEncoder);
        scheduler.robotPeriodic(0.02, 0.02);
        assertEquals(0.6, handle.percent, 1e-9);
    }

    @Test
    void recoveryDoesNotOverrideADeclaredManualDisable() {
        RecoverableMechanism mechanism = new RecoverableMechanism(true);
        RecordingMotor handle = new RecordingMotor(mechanism.motor);
        MechanismScheduler scheduler = MechanismScheduler.create()
                .motor(mechanism.motor, handle)
                .register(mechanism);
        scheduler.request(mechanism.run);

        scheduler.disableOwner(mechanism.firstEncoder);
        scheduler.recoverFailure(mechanism.firstEncoder);
        scheduler.robotPeriodic(0.0, 0.02);

        assertEquals(0.0, handle.percent, 1e-9);
    }

    private static final class RecoverableMechanism implements Mechanism {
        private final MotorDevice motor;
        private final EncoderDevice firstEncoder = EncoderDevice.of(EncoderKinds.CANCODER, 2);
        private final EncoderDevice secondEncoder = EncoderDevice.of(EncoderKinds.CANCODER, 3);
        private final Action run;

        private RecoverableMechanism(boolean disabled) {
            motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1).disabled(disabled);
            run = motor.percent(0.6);
        }
    }

    private static final class RecordingMotor implements MotorHandle {
        private final MotorDevice device;
        private double percent;

        private RecordingMotor(MotorDevice device) {
            this.device = device;
        }

        @Override public MotorDevice device() { return device; }
        @Override public void setPercentOutput(double percent) { this.percent = percent; }
    }
}
