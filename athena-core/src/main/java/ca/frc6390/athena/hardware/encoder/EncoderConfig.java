package ca.frc6390.athena.hardware.encoder;

import java.util.function.Consumer;

import ca.frc6390.athena.hardware.config.DeviceIdentityConfig;
import ca.frc6390.athena.hardware.config.DeviceIdentitySection;

/**
 * Vendor-agnostic encoder configuration.
 */
public class EncoderConfig extends DeviceIdentityConfig<EncoderType> {
    private double gearRatio = 1;
    private double offset = 0;
    private double conversion = 1;
    private double conversionOffset = 0;
    private double discontinuityPoint = Double.NaN;
    private double discontinuityRange = Double.NaN;

    /**
     */
    public EncoderConfig() {
    }

    public static EncoderConfig create() {
        return new EncoderConfig();
    }

    public static EncoderConfig create(EncoderType type) {
        EncoderConfig cfg = create();
        cfg.applyType(type);
        return cfg;
    }

    public static EncoderConfig create(EncoderType type, int id) {
        EncoderConfig cfg = create(type);
        cfg.applyId(id);
        return cfg;
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

    public HardwareSection hardware() {
        return new HardwareSection(this);
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

    public MeasurementSection measurement() {
        return new MeasurementSection(this);
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

    public double conversionOffset() {
        return conversionOffset;
    }

    public double discontinuityPoint() {
        return discontinuityPoint;
    }

    public double discontinuityRange() {
        return discontinuityRange;
    }

    public static final class HardwareSection extends DeviceIdentitySection<HardwareSection, EncoderType> {
        private HardwareSection(EncoderConfig owner) {
            super(owner::applyType, owner::applyId, owner::applyCanbus, owner::applyInverted);
        }

        @Override
        protected HardwareSection self() {
            return this;
        }
    }

    public static final class MeasurementSection {
        private final EncoderConfig owner;

        private MeasurementSection(EncoderConfig owner) {
            this.owner = owner;
        }

        public MeasurementSection gearRatio(double gearRatio) {
            owner.applyGearRatio(gearRatio);
            return this;
        }

        public MeasurementSection offset(double offset) {
            owner.applyOffset(offset);
            return this;
        }

        public MeasurementSection conversion(double conversion) {
            owner.applyConversion(conversion);
            return this;
        }

        public MeasurementSection conversionOffset(double conversionOffset) {
            owner.applyConversionOffset(conversionOffset);
            return this;
        }

        public MeasurementSection discontinuityPoint(double discontinuityPoint) {
            owner.applyDiscontinuityPoint(discontinuityPoint);
            return this;
        }

        public MeasurementSection discontinuityRange(double discontinuityRange) {
            owner.applyDiscontinuityRange(discontinuityRange);
            return this;
        }

        public MeasurementSection discontinuity(double discontinuityPoint, double discontinuityRange) {
            owner.applyDiscontinuity(discontinuityPoint, discontinuityRange);
            return this;
        }
    }

    @Override
    protected int normalizeId(int id) {
        return Math.abs(id);
    }

    @Override
    protected Boolean inferInvertedFromId(int id) {
        return id < 0;
    }

    private void applyGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
    }

    private void applyOffset(double offset) {
        this.offset = offset;
    }

    private void applyConversion(double conversion) {
        this.conversion = conversion;
    }

    private void applyConversionOffset(double conversionOffset) {
        this.conversionOffset = conversionOffset;
    }

    private void applyDiscontinuityPoint(double discontinuityPoint) {
        this.discontinuityPoint = discontinuityPoint;
    }

    private void applyDiscontinuityRange(double discontinuityRange) {
        this.discontinuityRange = discontinuityRange;
    }

    private void applyDiscontinuity(double discontinuityPoint, double discontinuityRange) {
        this.discontinuityPoint = discontinuityPoint;
        this.discontinuityRange = discontinuityRange;
    }
}
