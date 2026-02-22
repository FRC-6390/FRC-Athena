package ca.frc6390.athena.hardware.motor;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.hardware.encoder.Encoder;
import edu.wpi.first.math.controller.PIDController;
import org.junit.jupiter.api.Test;

final class MotorControllerAdapterTest {

    @Test
    void doesNotApplySecondInversionLayerToSpeedCommands() {
        FakeRawMotor raw = new FakeRawMotor();
        MotorControllerConfig config = MotorControllerConfig.create(TYPE, -3);
        MotorController adapter = new MotorControllerAdapter(raw, config);

        adapter.setSpeed(0.45);

        assertTrue(config.inverted());
        assertEquals(0.45, raw.lastSpeed, 1e-9);
    }

    @Test
    void setInvertedUpdatesConfigAndDelegatesToRawController() {
        FakeRawMotor raw = new FakeRawMotor();
        MotorControllerConfig config = MotorControllerConfig.create(TYPE, 3);
        MotorController adapter = new MotorControllerAdapter(raw, config);

        adapter.setInverted(true);
        assertTrue(config.inverted());
        assertTrue(raw.lastInverted);

        adapter.setInverted(false);
        assertFalse(config.inverted());
        assertFalse(raw.lastInverted);
    }

    private static final MotorControllerType TYPE = () -> "test";

    private static final class FakeRawMotor implements MotorController {
        private double lastSpeed = 0.0;
        private boolean lastInverted = false;

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

        @Override
        public void setInverted(boolean inverted) {
            lastInverted = inverted;
        }
    }
}
