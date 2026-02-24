package ca.frc6390.athena.hardware.encoder;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.hardware.examples.ImuEncoderEdgeExamples;
import org.junit.jupiter.api.Test;

final class EncoderEdgeBehaviorTest {

    @Test
    void adapterSetPositionHandlesZeroConversionAndGearRatio() {
        FakeRawEncoder raw = new FakeRawEncoder();
        EncoderConfig config = EncoderConfig.create(() -> "test", 1);
        config.measurement(m -> m.conversion(0.0).gearRatio(0.0));

        EncoderAdapter adapter = new EncoderAdapter(raw, config);
        adapter.setPosition(4.0);

        assertTrue(Double.isFinite(raw.position));
        assertEquals(4.0, raw.position, 1e-9);
        assertEquals(0.0, adapter.getPosition(), 1e-9);
    }

    @Test
    void discontinuityConfigurationWrapsAbsoluteRotations() {
        FakeRawEncoder raw = new FakeRawEncoder();
        raw.absolutePosition = 1.2;

        EncoderAdapter adapter = new EncoderAdapter(raw, EncoderConfig.create(() -> "test", 2));
        ImuEncoderEdgeExamples.configureEncoderDiscontinuity(adapter, 0.0, 1.0);
        adapter.update();

        assertEquals(0.2, adapter.getAbsoluteRotations(), 1e-9);
        assertEquals(72.0, adapter.getAbsoluteRotation2d().getDegrees(), 1e-9);
    }

    private static final class FakeRawEncoder implements Encoder {
        private double position;
        private double velocity;
        private double absolutePosition;

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
        public double getAbsolutePosition() {
            return absolutePosition;
        }

        @Override
        public void setSimulatedState(double rotations, double velocity) {
            position = rotations;
            this.velocity = velocity;
        }

        @Override
        public EncoderConfig getConfig() {
            return EncoderConfig.create(() -> "test", 0);
        }
    }
}
