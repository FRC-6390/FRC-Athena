package ca.frc6390.athena.devices;

public class EncoderConfig {
    
    public enum EncoderType {
        None,
        CTRETalonFXEncoder,
        CTRECANcoder,
        REVSparkFlexEncoder,
        REVSparkMaxEncoder,
        WPILibEncoder,
    }

    public EncoderType type = EncoderType.None;
    public int id;
    public String canbus = "rio";
    public double gearRatio = 1;
    public double offset = 0;
    public double conversion = 1;
    public boolean inverted = false;
    public double conversionOffset = 0;

    public EncoderConfig(){

    }

    public static EncoderConfig type(EncoderType type){
        EncoderConfig cfg = new EncoderConfig();
        cfg.type = type;
        return cfg;
    }

    public static EncoderConfig type(EncoderType type, int id){
        EncoderConfig cfg = new EncoderConfig();
        cfg.type = type;
        cfg.id = Math.abs(id);
        cfg.inverted = id < 0;
        return cfg;
    }

    public EncoderConfig setCanbus(String canbus){
        this.canbus = canbus;
        return this;
    }

    public EncoderConfig setOffset(double offset){
        this.offset = offset;
        return this;
    }

    public EncoderConfig setGearRatio(double gearRatio){
        this.gearRatio = gearRatio;
        return this;
    }

    public EncoderConfig setConversion(double conversion){
        this.conversion = conversion;
        return this;
    }

    public EncoderConfig setInverted(boolean inverted){
        this.inverted = inverted;
        return this;
    }

    public EncoderConfig setID(int id){
        this.id = id;
        return this;
    }

    public EncoderConfig setConversionOffset(double conversionOffset){
        this.conversionOffset = conversionOffset;
        return this;
    }

    public EncoderConfig setEncoderType(EncoderType type){
        this.type = type;
        return this;
    }

    public Encoder build(){
        return Encoder.fromConfig(this);
    }
    
}
