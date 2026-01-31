package ca.frc6390.athena.hardware.encoder;

/**
 * Vendor-agnostic encoder configuration.
 */
public class EncoderConfig {
    public EncoderType type;
    public int id;
    public String canbus = "rio";
    public double gearRatio = 1;
    public double offset = 0;
    public double conversion = 1;
    public boolean inverted = false;
    public double conversionOffset = 0;
    public double discontinuityPoint = Double.NaN;
    public double discontinuityRange = Double.NaN;

    public EncoderConfig() {
    }

    public static EncoderConfig type(EncoderType type) {
        EncoderConfig cfg = new EncoderConfig();
        cfg.type = type;
        return cfg;
    }

    public static EncoderConfig type(EncoderType type, int id) {
        EncoderConfig cfg = new EncoderConfig();
        cfg.type = type;
        cfg.id = Math.abs(id);
        cfg.inverted = id < 0;
        return cfg;
    }

    public EncoderConfig setCanbus(String canbus) {
        this.canbus = canbus;
        return this;
    }

    public EncoderConfig setOffset(double offset) {
        this.offset = offset;
        return this;
    }

    public EncoderConfig setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
        return this;
    }

    public EncoderConfig setConversion(double conversion) {
        this.conversion = conversion;
        return this;
    }

    public EncoderConfig setInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    public EncoderConfig setId(int id) {
        this.id = Math.abs(id);
        this.inverted = id < 0;
        return this;
    }

    public EncoderConfig setConversionOffset(double conversionOffset) {
        this.conversionOffset = conversionOffset;
        return this;
    }

    public EncoderConfig setDiscontinuityPoint(double discontinuityPoint) {
        this.discontinuityPoint = discontinuityPoint;
        return this;
    }

    public EncoderConfig setDiscontinuityRange(double discontinuityRange) {
        this.discontinuityRange = discontinuityRange;
        return this;
    }

    public EncoderConfig setDiscontinuity(double discontinuityPoint, double discontinuityRange) {
        this.discontinuityPoint = discontinuityPoint;
        this.discontinuityRange = discontinuityRange;
        return this;
    }

    public EncoderConfig setType(EncoderType type) {
        this.type = type;
        return this;
    }
}
