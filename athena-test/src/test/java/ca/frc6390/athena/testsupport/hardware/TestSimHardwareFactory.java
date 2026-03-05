package ca.frc6390.athena.testsupport.hardware;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderType;
import ca.frc6390.athena.hardware.factory.EncoderFactory;
import ca.frc6390.athena.hardware.factory.MotorControllerFactory;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import edu.wpi.first.math.controller.PIDController;

/**
 * Test-only hardware factory used by athena-test to run drivetrain simulation loops without
 * vendordep JNI requirements.
 */
public final class TestSimHardwareFactory implements MotorControllerFactory, EncoderFactory {

    public enum TestMotorType implements MotorControllerType {
        INSTANCE;

        @Override
        public String getKey() {
            return "test:sim-motor";
        }
    }

    public enum TestEncoderType implements EncoderType {
        INSTANCE;

        @Override
        public String getKey() {
            return "test:sim-encoder";
        }
    }

    @Override
    public boolean supports(MotorControllerType type) {
        return type != null && TestMotorType.INSTANCE.getKey().equals(type.getKey());
    }

    @Override
    public MotorController create(MotorControllerConfig config) {
        return new TestMotorController(config);
    }

    @Override
    public boolean supports(EncoderType type) {
        return type != null && TestEncoderType.INSTANCE.getKey().equals(type.getKey());
    }

    @Override
    public Encoder create(EncoderConfig config) {
        return new TestEncoder(config);
    }

    private static final class TestMotorController implements MotorController {
        private final MotorControllerConfig config;
        private final TestEncoder encoder;
        private double appliedPercent = 0.0;
        private double appliedVoltage = 0.0;
        private double currentLimit = 0.0;
        private boolean inverted = false;
        private MotorNeutralMode neutralMode = MotorNeutralMode.Coast;

        private TestMotorController(MotorControllerConfig config) {
            this.config = config != null ? config : new MotorControllerConfig();
            EncoderConfig encoderConfig = this.config.encoderConfig();
            if (encoderConfig == null) {
                encoderConfig = EncoderConfig.create(TestEncoderType.INSTANCE, this.config.id());
                this.config.encoder().config(encoderConfig);
            }
            this.encoder = new TestEncoder(encoderConfig);
            this.inverted = this.config.inverted();
            this.currentLimit = this.config.currentLimit();
            this.neutralMode = this.config.neutralMode();
        }

        @Override
        public int getId() {
            return config.id();
        }

        @Override
        public String getCanbus() {
            return config.canbus();
        }

        @Override
        public MotorControllerType getType() {
            return config.type();
        }

        @Override
        public void setSpeed(double percent) {
            appliedPercent = percent;
        }

        @Override
        public void setVoltage(double volts) {
            appliedVoltage = volts;
        }

        @Override
        public void setCurrentLimit(double amps) {
            currentLimit = amps;
        }

        @Override
        public void setPosition(double rotations) {
            encoder.setPosition(rotations);
        }

        @Override
        public void setVelocity(double rotationsPerSecond) {
            encoder.setSimulatedVelocity(rotationsPerSecond);
        }

        @Override
        public void setNeutralMode(MotorNeutralMode mode) {
            neutralMode = mode != null ? mode : MotorNeutralMode.Coast;
        }

        @Override
        public void setPid(PIDController pid) {
            // No-op for test simulation factory.
        }

        @Override
        public boolean isConnected() {
            return true;
        }

        @Override
        public double getTemperatureCelsius() {
            return 25.0;
        }

        @Override
        public double getCurrentAmps() {
            return Math.abs(appliedPercent) * 40.0;
        }

        @Override
        public double getAppliedVoltage() {
            return appliedVoltage;
        }

        @Override
        public Encoder getEncoder() {
            return encoder;
        }

        @Override
        public boolean isInverted() {
            return inverted;
        }

        @Override
        public void setInverted(boolean inverted) {
            this.inverted = inverted;
        }

        @Override
        public double getCurrentLimit() {
            return currentLimit;
        }

        @Override
        public MotorNeutralMode getNeutralMode() {
            return neutralMode;
        }

        @Override
        public MotorControllerConfig getConfig() {
            return config;
        }
    }

    private static final class TestEncoder implements Encoder {
        private final EncoderConfig config;
        private double position = 0.0;
        private double velocity = 0.0;
        private double absolutePosition = 0.0;
        private boolean inverted = false;
        private double conversion = 1.0;
        private double offset = 0.0;
        private double gearRatio = 1.0;
        private double conversionOffset = 0.0;
        private double discontinuityPoint = Double.NaN;
        private double discontinuityRange = Double.NaN;

        private TestEncoder(EncoderConfig config) {
            this.config = config != null ? config : EncoderConfig.create(TestEncoderType.INSTANCE, 0);
            this.inverted = this.config.inverted();
            this.conversion = this.config.conversion();
            this.offset = this.config.offset();
            this.gearRatio = this.config.gearRatio();
            this.conversionOffset = this.config.conversionOffset();
            this.discontinuityPoint = this.config.discontinuityPoint();
            this.discontinuityRange = this.config.discontinuityRange();
        }

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
            this.absolutePosition = position;
        }

        @Override
        public void setInverted(boolean inverted) {
            this.inverted = inverted;
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
        public void setGearRatio(double gearRatio) {
            this.gearRatio = gearRatio;
        }

        @Override
        public void setConversionOffset(double conversionOffset) {
            this.conversionOffset = conversionOffset;
        }

        @Override
        public void setDiscontinuityPoint(double discontinuityPoint) {
            this.discontinuityPoint = discontinuityPoint;
        }

        @Override
        public void setDiscontinuityRange(double discontinuityRange) {
            this.discontinuityRange = discontinuityRange;
        }

        @Override
        public double getAbsolutePosition() {
            return absolutePosition;
        }

        @Override
        public void setSimulatedPosition(double rotations) {
            this.position = rotations;
            this.absolutePosition = rotations;
        }

        @Override
        public void setSimulatedVelocity(double rotationsPerSecond) {
            this.velocity = rotationsPerSecond;
        }

        @Override
        public void setSimulatedState(double rotations, double velocity) {
            this.position = rotations;
            this.absolutePosition = rotations;
            this.velocity = velocity;
        }

        @Override
        public boolean supportsSimulation() {
            return true;
        }

        @Override
        public boolean isInverted() {
            return inverted;
        }

        @Override
        public double getConversion() {
            return conversion;
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
        public double getConversionOffset() {
            return conversionOffset;
        }

        @Override
        public double getDiscontinuityPoint() {
            return discontinuityPoint;
        }

        @Override
        public double getDiscontinuityRange() {
            return discontinuityRange;
        }

        @Override
        public double getRawAbsoluteValue() {
            return absolutePosition;
        }

        @Override
        public EncoderConfig getConfig() {
            return config;
        }
    }
}
