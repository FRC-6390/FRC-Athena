package ca.frc6390.athena.mechanisms;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerGroup;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotBase;
import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Test;

final class MechanismSimulationOutputTest {

    @Test
    void speedAndVoltageWritesAreVisibleThroughOutputViewInSimulation() {
        Assumptions.assumeTrue(RobotBase.isSimulation());

        FakeMotor motor = new FakeMotor();
        Mechanism mechanism = new Mechanism(
                new MotorControllerGroup(motor),
                new FakeEncoder(),
                false,
                false,
                new GenericLimitSwitch[0],
                false,
                0.02);

        mechanism.speed(0.4);
        assertEquals(0.4, mechanism.output(), 1e-9);
        assertEquals(0.4, motor.lastSpeed, 1e-9);

        mechanism.voltage(6.5);
        assertEquals(6.5, mechanism.output(), 1e-9);
        assertEquals(6.5, motor.lastVoltage, 1e-9);
    }

    private static final class FakeMotor implements MotorController {
        private static final MotorControllerType TYPE = () -> "test";
        private double lastSpeed = 0.0;
        private double lastVoltage = 0.0;

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
            lastVoltage = volts;
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
        @Override
        public double getPosition() {
            return 0.0;
        }

        @Override
        public double getVelocity() {
            return 0.0;
        }

        @Override
        public void setPosition(double position) {
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
