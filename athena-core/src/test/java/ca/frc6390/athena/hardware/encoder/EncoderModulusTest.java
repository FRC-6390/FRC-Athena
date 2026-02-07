package ca.frc6390.athena.hardware.encoder;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

final class EncoderModulusTest {

    @Test
    void wrapsPositionsIntoRange() {
        Encoder e = new FakeEncoder(450.0);
        assertEquals(90.0, e.getPositionModulus(0.0, 360.0), 1e-9);

        Encoder e2 = new FakeEncoder(-10.0);
        assertEquals(350.0, e2.getPositionModulus(0.0, 360.0), 1e-9);
    }

    private static final class FakeEncoder implements Encoder {
        private double position;

        private FakeEncoder(double position) {
            this.position = position;
        }

        @Override
        public double getPosition() {
            return position;
        }

        @Override
        public double getVelocity() {
            return 0.0;
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
            return new EncoderConfig();
        }
    }
}

