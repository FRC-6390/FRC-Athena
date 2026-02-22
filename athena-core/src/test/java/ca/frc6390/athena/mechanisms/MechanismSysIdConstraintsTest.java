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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Voltage;
import java.lang.reflect.Method;
import org.junit.jupiter.api.Test;

final class MechanismSysIdConstraintsTest {

    @Test
    void sysIdRespectsHardstopSuppression() {
        FakeMotor motor = new FakeMotor();
        FakeEncoder encoder = new FakeEncoder();
        GenericLimitSwitch hardstop = new GenericLimitSwitch(0, false);
        hardstop.setHardstop(true);
        hardstop.setBlockDirection(GenericLimitSwitch.BlockDirection.PositiveInput);
        hardstop.setSimulatedTriggered(true);
        try {
            Mechanism mechanism = createMechanism(motor, encoder, hardstop);
            invokeApplySysIdVoltage(mechanism, 6.0);
            assertEquals(0.0, motor.lastVoltage, 1e-9);
        } finally {
            hardstop.close();
        }
    }

    @Test
    void sysIdRespectsBoundsSuppression() {
        FakeMotor motor = new FakeMotor();
        FakeEncoder encoder = new FakeEncoder();
        encoder.position = 5.0;
        Mechanism mechanism = createMechanism(motor, encoder);
        mechanism.control().bounds(-5.0, 5.0);

        invokeApplySysIdVoltage(mechanism, 3.5);
        assertEquals(0.0, motor.lastVoltage, 1e-9);
    }

    @Test
    void sysIdRespectsMotionLimitSuppression() {
        FakeMotor motor = new FakeMotor();
        FakeEncoder encoder = new FakeEncoder();
        encoder.velocity = 2.6;
        Mechanism mechanism = createMechanism(motor, encoder);
        mechanism.motors().motionLimits(2.5, 0.0);

        invokeApplySysIdVoltage(mechanism, 4.0);
        assertEquals(0.0, motor.lastVoltage, 1e-9);
    }

    private static Mechanism createMechanism(FakeMotor motor, FakeEncoder encoder, GenericLimitSwitch... limitSwitches) {
        return new Mechanism(
                new MotorControllerGroup(motor),
                encoder,
                null,
                null,
                false,
                true,
                limitSwitches,
                false,
                0.02);
    }

    private static void invokeApplySysIdVoltage(Mechanism mechanism, double volts) {
        try {
            Method method = Mechanism.class.getDeclaredMethod("applySysIdVoltage", Voltage.class);
            method.setAccessible(true);
            method.invoke(mechanism, Units.Volts.of(volts));
        } catch (ReflectiveOperationException ex) {
            throw new RuntimeException(ex);
        }
    }

    private static final class FakeMotor implements MotorController {
        private static final MotorControllerType TYPE = () -> "test";
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
