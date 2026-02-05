package ca.frc6390.athena.hardware.encoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;

/**
 * Applies conversion, offsets, gear ratio, and inversion to a raw vendor encoder.
 */
public class EncoderAdapter implements Encoder {
    private final Encoder raw;
    private final EncoderConfig config;
    private double gearRatio = 1.0;
    private double offset = 0.0;
    private double conversion = 1.0;
    private double conversionOffset = 0.0;
    private double discontinuityPoint = Double.NaN;
    private double discontinuityRange = Double.NaN;
    private boolean inverted = false;
    private double shuffleboardPeriodSeconds = ca.frc6390.athena.core.RobotSendableSystem.getDefaultShuffleboardPeriodSeconds();

    public EncoderAdapter(Encoder raw, EncoderConfig config) {
        this.raw = raw;
        this.config = config;
        if (config != null) {
            this.gearRatio = config.gearRatio;
            this.offset = config.offset;
            this.conversion = config.conversion;
            this.conversionOffset = config.conversionOffset;
            this.discontinuityPoint = config.discontinuityPoint;
            this.discontinuityRange = config.discontinuityRange;
            this.inverted = config.inverted;
        }
    }

    public static Encoder wrap(Encoder raw, EncoderConfig config) {
        if (raw == null) {
            return null;
        }
        if (raw instanceof EncoderAdapter) {
            return raw;
        }
        return new EncoderAdapter(raw, config);
    }

    @Override
    public double getPosition() {
        return getRotations() * conversion + conversionOffset;
    }

    @Override
    public double getVelocity() {
        return getRate() * conversion;
    }

    @Override
    public void setPosition(double position) {
        double safeConversion = conversion != 0.0 ? conversion : 1.0;
        double safeGearRatio = gearRatio != 0.0 ? gearRatio : 1.0;
        double rawPosition = ((position - conversionOffset) / safeConversion) / safeGearRatio + offset;
        if (!Double.isFinite(rawPosition)) {
            rawPosition = 0.0;
        }
        raw.setPosition(rawPosition);
    }

    @Override
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
        if (config != null) {
            config.inverted = inverted;
        }
    }

    @Override
    public void setConversion(double conversion) {
        this.conversion = conversion;
        if (config != null) {
            config.conversion = conversion;
        }
    }

    @Override
    public void setOffset(double offset) {
        this.offset = offset;
        if (config != null) {
            config.offset = offset;
        }
    }

    @Override
    public void setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
        if (config != null) {
            config.gearRatio = gearRatio;
        }
    }

    @Override
    public void setConversionOffset(double conversionOffset) {
        this.conversionOffset = conversionOffset;
        if (config != null) {
            config.conversionOffset = conversionOffset;
        }
    }

    @Override
    public void setDiscontinuityPoint(double discontinuityPoint) {
        this.discontinuityPoint = discontinuityPoint;
        if (config != null) {
            config.discontinuityPoint = discontinuityPoint;
        }
    }

    @Override
    public void setDiscontinuityRange(double discontinuityRange) {
        this.discontinuityRange = discontinuityRange;
        if (config != null) {
            config.discontinuityRange = discontinuityRange;
        }
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
    public double getDiscontinuityPoint() {
        return discontinuityPoint;
    }

    @Override
    public double getDiscontinuityRange() {
        return discontinuityRange;
    }

    @Override
    public double getGearRatio() {
        return gearRatio;
    }

    @Override
    public boolean isInverted() {
        return inverted;
    }

    @Override
    public double getRotations() {
        return (getRawPosition() - offset) * gearRatio;
    }

    @Override
    public double getAbsolutePosition() {
        return getAbsoluteRotations() * conversion + conversionOffset;
    }

    @Override
    public double getAbsoluteRotations() {
        double rotations = (getRawAbsolutePosition() - offset) * gearRatio;
        return applyDiscontinuity(rotations);
    }

    @Override
    public Rotation2d getRotation2d() {
        return Rotation2d.fromRotations(getRotations());
    }

    @Override
    public Rotation2d getAbsoluteRotation2d() {
        return Rotation2d.fromRotations(getAbsoluteRotations());
    }

    @Override
    public double getRate() {
        return getRawVelocity() * gearRatio;
    }

    @Override
    public void setRotations(double rotations) {
        setPosition(rotations * conversion + conversionOffset);
    }

    @Override
    public double getRawAbsoluteValue() {
        return getRawAbsolutePosition();
    }

    @Override
    public Encoder update() {
        raw.update();
        return this;
    }

    @Override
    public double getShuffleboardPeriodSeconds() {
        return shuffleboardPeriodSeconds;
    }

    @Override
    public void setShuffleboardPeriodSeconds(double periodSeconds) {
        if (!Double.isFinite(periodSeconds)) {
            return;
        }
        shuffleboardPeriodSeconds = periodSeconds;
        raw.setShuffleboardPeriodSeconds(periodSeconds);
    }

    @Override
    public void setSimulatedPosition(double rotations) {
        raw.setSimulatedPosition(rotations);
    }

    @Override
    public void setSimulatedVelocity(double rotationsPerSecond) {
        raw.setSimulatedVelocity(rotationsPerSecond);
    }

    @Override
    public void setSimulatedState(double rotations, double velocity) {
        raw.setSimulatedState(rotations, velocity);
    }

    @Override
    public boolean supportsSimulation() {
        return raw.supportsSimulation();
    }

    @Override
    public EncoderConfig getConfig() {
        return config != null ? config : raw.getConfig();
    }

    private double getRawPosition() {
        double rawPosition = raw.getPosition();
        return inverted ? -rawPosition : rawPosition;
    }

    private double getRawAbsolutePosition() {
        double rawPosition = raw.getAbsolutePosition();
        return inverted ? -rawPosition : rawPosition;
    }

    private double getRawVelocity() {
        double rawVelocity = raw.getVelocity();
        return inverted ? -rawVelocity : rawVelocity;
    }

    private double applyDiscontinuity(double value) {
        if (!Double.isFinite(discontinuityPoint) || !Double.isFinite(discontinuityRange) || discontinuityRange <= 0.0) {
            return value;
        }
        double min = discontinuityPoint - (discontinuityRange / 2.0);
        double max = discontinuityPoint + (discontinuityRange / 2.0);
        if (!Double.isFinite(min) || !Double.isFinite(max) || min == max) {
            return value;
        }
        double wrapped = MathUtil.inputModulus(value, min, max);
        return Double.isFinite(wrapped) ? wrapped : value;
    }
}
