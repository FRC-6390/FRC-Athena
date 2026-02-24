package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.math.controller.PIDController;
import org.junit.jupiter.api.Test;

final class MechanismEmergencyStopRecoveryTest {

    @Test
    void clearsConnectivityEmergencyStopAfterRecovery() {
        FakeMotor motor = new FakeMotor();
        Mechanism mechanism = createMechanism(motor);

        mechanism.update();
        assertFalse(mechanism.emergencyStopped());

        motor.setConnected(false);
        mechanism.update();
        assertTrue(mechanism.emergencyStopped());
        assertTrue(motor.stopCalls > 0);

        motor.setConnected(true);
        mechanism.update();
        assertFalse(mechanism.emergencyStopped());
    }

    @Test
    void manualEmergencyStopRemainsLatchedUntilCleared() {
        FakeMotor motor = new FakeMotor();
        Mechanism mechanism = createMechanism(motor);

        mechanism.control().emergencyStop(true);
        mechanism.update();
        assertTrue(mechanism.emergencyStopped());

        motor.setConnected(false);
        mechanism.update();
        assertTrue(mechanism.emergencyStopped());

        motor.setConnected(true);
        mechanism.update();
        assertTrue(mechanism.emergencyStopped());

        mechanism.control().emergencyStop(false);
        mechanism.update();
        assertFalse(mechanism.emergencyStopped());
    }

    private static Mechanism createMechanism(FakeMotor motor) {
        return new Mechanism(
                new MotorControllerGroup(motor),
                null,
                null,
                null,
                false,
                false,
                new GenericLimitSwitch[0],
                false,
                0.02);
    }

    private static final class FakeMotor implements MotorController {
        private static final MotorControllerType TYPE = () -> "test";
        private boolean connected = true;
        private int stopCalls = 0;

        void setConnected(boolean connected) {
            this.connected = connected;
        }

        @Override
        public int getId() {
            return 1;
        }

        @Override
        public String getCanbus() {
            return "rio";
        }

        @Override
        public MotorControllerType getType() {
            return TYPE;
        }

        @Override
        public void setSpeed(double percent) {
        }

        @Override
        public void setVoltage(double volts) {
        }

        @Override
        public void setCurrentLimit(double amps) {
        }

        @Override
        public void setPosition(double rotations) {
        }

        @Override
        public void setVelocity(double rotationsPerSecond) {
        }

        @Override
        public void setNeutralMode(MotorNeutralMode mode) {
        }

        @Override
        public void setPid(PIDController pid) {
        }

        @Override
        public boolean isConnected() {
            return connected;
        }

        @Override
        public double getTemperatureCelsius() {
            return 0.0;
        }

        @Override
        public Encoder getEncoder() {
            return null;
        }

        @Override
        public void stopMotor() {
            stopCalls++;
        }
    }
}
