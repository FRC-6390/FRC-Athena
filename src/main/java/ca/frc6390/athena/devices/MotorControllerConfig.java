package ca.frc6390.athena.devices;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import ca.frc6390.athena.devices.EncoderConfig.EncoderType;
import edu.wpi.first.math.controller.PIDController;

public class MotorControllerConfig {
    
     public enum MotorControllerType {
        CTRETalonFX(EncoderType.CTRETalonFXEncoder),
        REVSparkMaxBrushed(EncoderType.REVSparkMaxEncoder),
        REVSparkMaxBrushless(EncoderType.REVSparkMaxEncoder),
        REVSparkFlexBrushed(EncoderType.REVSparkFlexEncoder),
        REVSparkFlexBrushless(EncoderType.REVSparkFlexEncoder);

        private EncoderType encoder;
        MotorControllerType(EncoderType encoder){
            this.encoder = encoder;
        }


        public EncoderType getEncoder(){
            return encoder;
        }

        public MotorControllerConfig config(int id){
            return new MotorControllerConfig(this, id);
        }
    }

    public enum MotorNeutralMode {
        Coast(NeutralModeValue.Coast, IdleMode.kCoast),
        Brake(NeutralModeValue.Brake, IdleMode.kBrake);

        private final NeutralModeValue ctre;
        private final IdleMode rev;

        MotorNeutralMode(NeutralModeValue ctre, IdleMode rev){
            this.ctre = ctre;
            this.rev = rev;
        }

        public NeutralModeValue asCTRE(){
            return ctre;
        }

        public IdleMode asREV(){
            return rev;
        }

        public boolean asBoolean(){
            return ctre == NeutralModeValue.Brake;
        }

        public static MotorNeutralMode fromBoolean(boolean value) {
            return value ? MotorNeutralMode.Brake : MotorNeutralMode.Coast;
        }

        public static MotorNeutralMode fromREV(IdleMode mode){
            switch (mode) {
                case kBrake:
                return MotorNeutralMode.Brake;            
                case kCoast:
                return MotorNeutralMode.Coast;
                default:
                return null;
            }
        }

        public static MotorNeutralMode fromCTRE(NeutralModeValue mode){
            switch (mode) {
                case Brake:
                return MotorNeutralMode.Brake;            
                case Coast:
                return MotorNeutralMode.Coast;
                default:
                return null;
            }
        }
    }

    public MotorControllerType type;
    public int id;
    public String canbus = "rio";
    public double currentLimit = 40;
    public boolean inverted = false;
    public EncoderConfig encoderConfig;
    public MotorNeutralMode neutralMode = MotorNeutralMode.Coast;
    public PIDController pid;

    public MotorControllerConfig(MotorControllerType type, int id){
        this.id = Math.abs(id);
        this.inverted = id < 0;
        this.type = type;
        this.encoderConfig = EncoderConfig.type(type.getEncoder()).setID(id);
    }

    public MotorControllerConfig setCanbus(String canbus){
        this.canbus = canbus;
        return this;
    }

    public MotorControllerConfig setEncoderConfig(EncoderConfig encoderConfig){
        this.encoderConfig = encoderConfig;
        return this;
    }

    public MotorControllerConfig setCurrentLimit(double currentLimit){
        this.currentLimit = currentLimit;
        return this;
    }

    public MotorControllerConfig setInverted(boolean inverted){
        this.inverted = inverted;
        return this;
    }

    public MotorControllerConfig setNeutralMode(MotorNeutralMode neutralMode){
        this.neutralMode = neutralMode;
        return this;
    }

    public MotorControllerConfig setID(int id){
        this.id = id;
        return this;
    }

    public MotorControllerConfig setPid(double p, double i, double d) {
        return setPid(new PIDController(p, i, d));
    }

    public MotorControllerConfig setPid(PIDController pid) {
        this.pid = pid;
        return this;
    }


}
