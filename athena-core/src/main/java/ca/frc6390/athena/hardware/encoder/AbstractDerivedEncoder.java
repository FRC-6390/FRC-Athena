package ca.frc6390.athena.hardware.encoder;

/**
 * Base class for derived encoders that expose a single primary signal.
 */
abstract class AbstractDerivedEncoder implements Encoder {
    enum OutputKind {
        POSITION,
        VELOCITY
    }

    private final EncoderConfig config;
    private final OutputKind outputKind;
    private final boolean absoluteCapable;
    protected double cachedValue;

    protected AbstractDerivedEncoder(EncoderConfig config, OutputKind outputKind, boolean absoluteCapable) {
        this.config = config != null ? config : EncoderConfig.create();
        this.outputKind = outputKind;
        this.absoluteCapable = absoluteCapable;
    }

    @Override
    public double getPosition() {
        return outputKind == OutputKind.POSITION ? cachedValue : 0.0;
    }

    @Override
    public double getVelocity() {
        return outputKind == OutputKind.VELOCITY ? cachedValue : 0.0;
    }

    @Override
    public double getAbsolutePosition() {
        if (absoluteCapable) {
            return cachedValue;
        }
        return getPosition();
    }

    @Override
    public void setPosition(double position) {
        throw new UnsupportedOperationException("derived encoder sources are read-only");
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
        return config;
    }

    protected static double nearestEquivalent(double value, double reference, double span) {
        if (!Double.isFinite(span) || span <= 0.0) {
            return value;
        }
        return reference + edu.wpi.first.math.MathUtil.inputModulus(value - reference, -span / 2.0, span / 2.0);
    }
}
