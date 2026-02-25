package ca.frc6390.athena.rev.encoder;

import static org.junit.jupiter.api.Assertions.assertEquals;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import org.junit.jupiter.api.Test;

final class RevEncoderBehaviorTest {

    @Test
    void separatesRelativeAndAbsolutePositionSources() {
        FakeRelativeEncoder relative = new FakeRelativeEncoder(12.0, 600.0);
        FakeAbsoluteEncoder absolute = new FakeAbsoluteEncoder(0.25, 30.0);
        RevEncoder encoder = new RevEncoder(relative, absolute, EncoderConfig.create(() -> "test", 1));

        assertEquals(12.0, encoder.getPosition(), 1e-9);
        assertEquals(0.25, encoder.getAbsolutePosition(), 1e-9);
        assertEquals(10.0, encoder.getVelocity(), 1e-9);
        assertEquals(0.25, encoder.getRawAbsoluteValue(), 1e-9);
    }

    @Test
    void fallsBackToAbsoluteWhenRelativeEncoderMissing() {
        FakeAbsoluteEncoder absolute = new FakeAbsoluteEncoder(0.75, 120.0);
        RevEncoder encoder = new RevEncoder(null, absolute, EncoderConfig.create(() -> "test", 2));

        assertEquals(0.75, encoder.getPosition(), 1e-9);
        assertEquals(0.75, encoder.getAbsolutePosition(), 1e-9);
        assertEquals(2.0, encoder.getVelocity(), 1e-9);
    }

    private static final class FakeRelativeEncoder implements RelativeEncoder {
        private double position;
        private double velocityRpm;

        private FakeRelativeEncoder(double position, double velocityRpm) {
            this.position = position;
            this.velocityRpm = velocityRpm;
        }

        @Override
        public double getPosition() {
            return position;
        }

        @Override
        public double getVelocity() {
            return velocityRpm;
        }

        @Override
        public REVLibError setPosition(double position) {
            this.position = position;
            return REVLibError.kOk;
        }
    }

    private static final class FakeAbsoluteEncoder implements AbsoluteEncoder {
        private final double position;
        private final double velocityRpm;

        private FakeAbsoluteEncoder(double position, double velocityRpm) {
            this.position = position;
            this.velocityRpm = velocityRpm;
        }

        @Override
        public double getPosition() {
            return position;
        }

        @Override
        public double getVelocity() {
            return velocityRpm;
        }
    }
}
