package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertNotEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.math.controller.PIDController;
import org.junit.jupiter.api.Test;

final class MechanismDocsExamplesTest {

    @Test
    void disableAllHooksAndControlLoopsMatchesDocsExample() {
        FakeMotor motor = new FakeMotor();
        Mechanism mechanism = createMechanism(motor, new FakeEncoder(), null);

        assertTrue(mechanism.hooksEnabled());
        assertTrue(mechanism.controlLoopsEnabled());

        mechanism.control().disableAllHooksAndControlLoops();

        assertFalse(mechanism.hooksEnabled());
        assertFalse(mechanism.controlLoopsEnabled());
    }

    @Test
    void testModeSuppressesPidOutput() {
        FakeMotor motor = new FakeMotor();
        FakeEncoder encoder = new FakeEncoder();
        Mechanism mechanism = createMechanism(motor, encoder, new PIDController(1.0, 0.0, 0.0));
        mechanism.setpoint(1.0);

        setRobotMode(mechanism, "TELE");
        mechanism.update();
        assertNotEquals(0.0, motor.lastSpeed, 1e-9);

        setRobotMode(mechanism, "TEST");
        mechanism.update();
        assertEquals(0.0, motor.lastSpeed, 1e-9);
    }

    private static void setRobotMode(Mechanism mechanism, String mode) {
        mechanism.setRobotModeForTest(mode);
    }

    private static Mechanism createMechanism(FakeMotor motor, FakeEncoder encoder, PIDController pid) {
        return new Mechanism(
                new MotorControllerGroup(motor),
                encoder,
                pid,
                null,
                false,
                false,
                new GenericLimitSwitch[0],
                false,
                0.02);
    }

    private static final class FakeMotor implements MotorController {
        private static final MotorControllerType TYPE = () -> "test";
        private double lastSpeed = 0.0;

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
            lastSpeed = percent;
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
            return true;
        }

        @Override
        public double getTemperatureCelsius() {
            return 0.0;
        }

        @Override
        public Encoder getEncoder() {
            return null;
        }
    }

    private static final class FakeEncoder implements Encoder {
        private double position = 0.0;
        private double velocity = 0.0;

        @Override
        public double getPosition() {
            return position;
        }

        @Override
        public double getVelocity() {
            return velocity;
        }

        @Override
        public void setPosition(double position) {
            this.position = position;
        }

        @Override
        public void setInverted(boolean inverted) {
        }

        @Override
        public void setConversion(double conversion) {
        }

        @Override
        public void setOffset(double offset) {
        }

        @Override
        public EncoderConfig getConfig() {
            return null;
        }
    }
}
