package ca.frc6390.athena.mechanisms.sim;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import org.junit.jupiter.api.Test;

final class MechanismSimUtilTest {

    @Test
    void applyEncoderStateHonorsConversionAndOffset() {
        FakeEncoder encoder = new FakeEncoder()
                .withConversion(360.0)
                .withOffset(0.7705078125)
                .withConversionOffset(0.0)
                .withGearRatio(1.0);

        double targetPos = 90.0;
        double targetVel = -15.0;
        MechanismSimUtil.applyEncoderState(encoder, targetPos, targetVel);

        assertEquals(targetPos, encoder.getPosition(), 1e-6);
        assertEquals(targetVel, encoder.getVelocity(), 1e-6);
    }

    @Test
    void applyEncoderStateAppliesConversionOffsetOnce() {
        FakeEncoder encoder = new FakeEncoder()
                .withConversion(360.0)
                .withOffset(0.5)
                .withConversionOffset(15.0)
                .withGearRatio(1.0);

        double targetPos = 90.0;
        MechanismSimUtil.applyEncoderState(encoder, targetPos, 0.0);

        double expectedRaw = (targetPos - 15.0) / 360.0 - 0.5;
        assertEquals(expectedRaw, encoder.getSimPosition(), 1e-6);
        assertEquals(targetPos, encoder.getPosition(), 1e-6);
    }

    @Test
    void applyEncoderStateMatchesWristOffsetConfig() {
        FakeEncoder encoder = new FakeEncoder()
                .withConversion(360.0)
                .withOffset(0.2587890625)
                .withConversionOffset(0.0)
                .withGearRatio(1.0);

        double targetPos = 60.0;
        MechanismSimUtil.applyEncoderState(encoder, targetPos, 0.0);

        assertEquals(targetPos, encoder.getPosition(), 1e-6);
    }

    private static final class FakeEncoder implements Encoder {
        private final EncoderConfig config = new EncoderConfig();
        private double conversion = 1.0;
        private double conversionOffset = 0.0;
        private double offset = 0.0;
        private double gearRatio = 1.0;
        private double simPosition = 0.0;
        private double simVelocity = 0.0;

        FakeEncoder withConversion(double conversion) {
            this.conversion = conversion;
            return this;
        }

        FakeEncoder withConversionOffset(double conversionOffset) {
            this.conversionOffset = conversionOffset;
            return this;
        }

        FakeEncoder withOffset(double offset) {
            this.offset = offset;
            return this;
        }

        FakeEncoder withGearRatio(double gearRatio) {
            this.gearRatio = gearRatio;
            return this;
        }

        @Override
        public double getPosition() {
            return ((simPosition + offset) * gearRatio) * conversion + conversionOffset;
        }

        @Override
        public double getVelocity() {
            return simVelocity * gearRatio * conversion;
        }

        @Override
        public void setPosition(double position) {
            simPosition = ((position - conversionOffset) / conversion) / gearRatio - offset;
        }

        @Override
        public void setInverted(boolean inverted) {
            // no-op
        }

        @Override
        public void setConversion(double conversion) {
            this.conversion = conversion;
        }

        @Override
        public void setOffset(double offset) {
            this.offset = offset;
        }

        @Override
        public double getConversion() {
            return conversion;
        }

        @Override
        public double getConversionOffset() {
            return conversionOffset;
        }

        @Override
        public double getOffset() {
            return offset;
        }

        @Override
        public double getGearRatio() {
            return gearRatio;
        }

        @Override
        public void setGearRatio(double gearRatio) {
            this.gearRatio = gearRatio;
        }

        @Override
        public void setConversionOffset(double conversionOffset) {
            this.conversionOffset = conversionOffset;
        }

        @Override
        public void setSimulatedState(double rotations, double velocity) {
            this.simPosition = rotations;
            this.simVelocity = velocity;
        }

        double getSimPosition() {
            return simPosition;
        }

        @Override
        public boolean supportsSimulation() {
            return true;
        }

        @Override
        public EncoderConfig getConfig() {
            return config;
        }
    }
}
