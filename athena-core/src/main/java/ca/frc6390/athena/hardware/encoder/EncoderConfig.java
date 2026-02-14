package ca.frc6390.athena.hardware.encoder;

import java.util.function.Consumer;

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

    /**
     */
    public EncoderConfig() {
    }

    public static EncoderConfig create() {
        return new EncoderConfig();
    }

    public static EncoderConfig create(EncoderType type) {
        EncoderConfig cfg = create();
        cfg.type = type;
        return cfg;
    }

    public static EncoderConfig create(EncoderType type, int id) {
        EncoderConfig cfg = create(type);
        cfg.setId(id);
        return cfg;
    }

    /**
     */
    public static EncoderConfig type(EncoderType type) {
        return create(type);
    }

    /**
     */
    public static EncoderConfig type(EncoderType type, int id) {
        return create(type, id);
    }

    /**
     * Sectioned fluent API for basic hardware identity settings.
     */
    public EncoderConfig hardware(Consumer<HardwareSection> section) {
        if (section != null) {
            section.accept(new HardwareSection(this));
        }
        return this;
    }

    /**
     * Sectioned fluent API for conversion/normalization settings.
     */
    public EncoderConfig measurement(Consumer<MeasurementSection> section) {
        if (section != null) {
            section.accept(new MeasurementSection(this));
        }
        return this;
    }

    /**
     */
    public EncoderConfig setCanbus(String canbus) {
        this.canbus = canbus;
        return this;
    }

    /**
     */
    public EncoderConfig setOffset(double offset) {
        this.offset = offset;
        return this;
    }

    /**
     */
    public EncoderConfig setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
        return this;
    }

    /**
     */
    public EncoderConfig setConversion(double conversion) {
        this.conversion = conversion;
        return this;
    }

    /**
     */
    public EncoderConfig setInverted(boolean inverted) {
        this.inverted = inverted;
        return this;
    }

    /**
     */
    public EncoderConfig setId(int id) {
        this.id = Math.abs(id);
        this.inverted = id < 0;
        return this;
    }

    /**
     */
    public EncoderConfig setConversionOffset(double conversionOffset) {
        this.conversionOffset = conversionOffset;
        return this;
    }

    /**
     */
    public EncoderConfig setDiscontinuityPoint(double discontinuityPoint) {
        this.discontinuityPoint = discontinuityPoint;
        return this;
    }

    /**
     */
    public EncoderConfig setDiscontinuityRange(double discontinuityRange) {
        this.discontinuityRange = discontinuityRange;
        return this;
    }

    /**
     */
    public EncoderConfig setDiscontinuity(double discontinuityPoint, double discontinuityRange) {
        this.discontinuityPoint = discontinuityPoint;
        this.discontinuityRange = discontinuityRange;
        return this;
    }

    /**
     */
    public EncoderConfig setType(EncoderType type) {
        this.type = type;
        return this;
    }

    public EncoderType type() {
        return type;
    }

    public int id() {
        return id;
    }

    public String canbus() {
        return canbus;
    }

    public double gearRatio() {
        return gearRatio;
    }

    public double offset() {
        return offset;
    }

    public double conversion() {
        return conversion;
    }

    public boolean inverted() {
        return inverted;
    }

    public double conversionOffset() {
        return conversionOffset;
    }

    public double discontinuityPoint() {
        return discontinuityPoint;
    }

    public double discontinuityRange() {
        return discontinuityRange;
    }

    public static final class HardwareSection {
        private final EncoderConfig owner;

        private HardwareSection(EncoderConfig owner) {
            this.owner = owner;
        }

        public HardwareSection type(EncoderType type) {
            owner.setType(type);
            return this;
        }

        public HardwareSection id(int id) {
            owner.setId(id);
            return this;
        }

        public HardwareSection canbus(String canbus) {
            owner.setCanbus(canbus);
            return this;
        }

        public HardwareSection inverted(boolean inverted) {
            owner.setInverted(inverted);
            return this;
        }
    }

    public static final class MeasurementSection {
        private final EncoderConfig owner;

        private MeasurementSection(EncoderConfig owner) {
            this.owner = owner;
        }

        public MeasurementSection gearRatio(double gearRatio) {
            owner.setGearRatio(gearRatio);
            return this;
        }

        public MeasurementSection offset(double offset) {
            owner.setOffset(offset);
            return this;
        }

        public MeasurementSection conversion(double conversion) {
            owner.setConversion(conversion);
            return this;
        }

        public MeasurementSection conversionOffset(double conversionOffset) {
            owner.setConversionOffset(conversionOffset);
            return this;
        }

        public MeasurementSection discontinuityPoint(double discontinuityPoint) {
            owner.setDiscontinuityPoint(discontinuityPoint);
            return this;
        }

        public MeasurementSection discontinuityRange(double discontinuityRange) {
            owner.setDiscontinuityRange(discontinuityRange);
            return this;
        }

        public MeasurementSection discontinuity(double discontinuityPoint, double discontinuityRange) {
            owner.setDiscontinuity(discontinuityPoint, discontinuityRange);
            return this;
        }
    }
}
