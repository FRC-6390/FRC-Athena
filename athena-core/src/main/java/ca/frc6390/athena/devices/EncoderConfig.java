package ca.frc6390.athena.devices;

/**
 * Mutable configuration object that defines how an encoder or sensor is wired to the robot. Tracks
 * CAN IDs, DIO ports, conversion factors, and inversion so mechanisms can share a common builder
 * pattern.
 */
public class EncoderConfig {
    
    /**
     * Supported encoder hardware families. Some controllers (e.g., TalonFX) expose an integrated
     * encoder, while others rely on external sensors.
     */
    public enum EncoderType {
        None,
        CTRETalonFXEncoder,
        CTRECANcoder,
        REVSparkFlexEncoder,
        REVSparkMaxEncoder,
        WPILibEncoder,
    }

    /** Encoder family being configured. */
    public EncoderType type = EncoderType.None;
    /** CAN ID or DIO port used by the encoder (negative values imply inversion). */
    public int id;
    /** CAN bus used to communicate with the sensor (ignored for non-CAN devices). */
    public String canbus = "rio";
    /** Gear ratio from sensor shaft to mechanism output. */
    public double gearRatio = 1;
    /** Raw sensor offset added to the measured position. */
    public double offset = 0;
    /** Scalar that converts sensor units into mechanism units. */
    public double conversion = 1;
    /** Whether the sensor measurement should be inverted. */
    public boolean inverted = false;
    /** Additional term added to the conversion result (useful for calibrating absolute encoders). */
    public double conversionOffset = 0;

    public EncoderConfig(){

    }

    /**
     * Creates an encoder configuration using the provided type with no ID assigned.
     */
    public static EncoderConfig type(EncoderType type){
        EncoderConfig cfg = new EncoderConfig();
        cfg.type = type;
        return cfg;
    }

    /**
     * Creates an encoder configuration using the provided type and hardware ID. Negative IDs mark
     * the encoder as inverted.
     */
    public static EncoderConfig type(EncoderType type, int id){
        EncoderConfig cfg = new EncoderConfig();
        cfg.type = type;
        cfg.id = Math.abs(id);
        cfg.inverted = id < 0;
        return cfg;
    }

    /**
     * Sets the CAN bus used to communicate with the sensor.
     */
    public EncoderConfig setCanbus(String canbus){
        this.canbus = canbus;
        return this;
    }

    /**
     * Sets the raw offset applied to the encoder reading.
     */
    public EncoderConfig setOffset(double offset){
        this.offset = offset;
        return this;
    }

    /**
     * Declares the gear ratio between the encoder and the mechanism output.
     */
    public EncoderConfig setGearRatio(double gearRatio){
        this.gearRatio = gearRatio;
        return this;
    }

    /**
     * Sets the conversion factor used to translate sensor units into mechanism units.
     */
    public EncoderConfig setConversion(double conversion){
        this.conversion = conversion;
        return this;
    }

    /**
     * Marks the encoder as inverted.
     */
    public EncoderConfig setInverted(boolean inverted){
        this.inverted = inverted;
        return this;
    }

    /**
     * Sets the sensor's hardware ID (CAN ID, DIO port, etc.).
     */
    public EncoderConfig setID(int id){
        this.id = id;
        return this;
    }

    /**
     * Sets an additional additive term applied after the primary conversion multiplier.
     */
    public EncoderConfig setConversionOffset(double conversionOffset){
        this.conversionOffset = conversionOffset;
        return this;
    }

    /**
     * Updates the encoder type.
     */
    public EncoderConfig setEncoderType(EncoderType type){
        this.type = type;
        return this;
    }

    /**
     * Constructs a runtime {@link Encoder} instance using this configuration.
     */
    public Encoder build(){
        return Encoder.fromConfig(this);
    }
    
}
