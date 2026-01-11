package ca.frc6390.athena.devices;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import ca.frc6390.athena.devices.EncoderConfig.EncoderType;
import edu.wpi.first.math.controller.PIDController;

/**
 * Mutable configuration object that describes how to initialize a motor controller (CAN ID, bus,
 * current limit, PID, etc.). The builder-style setters allow fluent chaining when assembling
 * mechanism configurations.
 */
public class MotorControllerConfig {
    
     public enum MotorControllerType {
        CTRETalonFX(EncoderType.CTRETalonFXEncoder),
        REVSparkMaxBrushed(EncoderType.REVSparkMaxEncoder),
        REVSparkMaxBrushless(EncoderType.REVSparkMaxEncoder),
        REVSparkFlexBrushed(EncoderType.REVSparkFlexEncoder),
        REVSparkFlexBrushless(EncoderType.REVSparkFlexEncoder);

        /** Default encoder type that ships with the selected controller. */
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

        /** Native CTRE neutral mode that corresponds to the enum. */
        private final NeutralModeValue ctre;
        /** Native REV neutral mode that corresponds to the enum. */
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

    /** Underlying controller family (TalonFX, SparkMax, etc.). */
    public MotorControllerType type;
    /** CAN ID configured on the device (negative IDs indicate an inverted motor). */
    public int id;
    /** CAN bus name shared by the controller and its sensor. */
    public String canbus = "rio";
    /** Current limit in amps applied to the controller. */
   public double currentLimit = 40;
    /** Whether the motor output should be inverted. */
    public boolean inverted = false;
    /** Encoder configuration associated with this controller. */
    public EncoderConfig encoderConfig;
    /** Neutral behavior applied to the controller (brake/coast). */
    public MotorNeutralMode neutralMode = MotorNeutralMode.Coast;
    /** Optional PID controller instance for onboard closed-loop control. */
    public PIDController pid;

    /**
     * Creates a new configuration for the provided controller type and CAN ID. Negative IDs mark the
     * motor as inverted and flip the encoder direction.
     */
    public MotorControllerConfig(MotorControllerType type, int id){
        this.id = Math.abs(id);
        this.inverted = id < 0;
        this.type = type;
        this.encoderConfig = EncoderConfig.type(type.getEncoder()).setID(id);
        syncEncoderCanbus();
    }

    /**
     * Sets the CAN bus used by the motor controller (and integrated encoder when applicable).
     */
    public MotorControllerConfig setCanbus(String canbus){
        this.canbus = canbus;
        syncEncoderCanbus();
        return this;
    }

    /**
     * Overrides the encoder configuration used for feedback.
     */
    public MotorControllerConfig setEncoderConfig(EncoderConfig encoderConfig){
        this.encoderConfig = encoderConfig;
        syncEncoderCanbus();
        return this;
    }

    /**
     * Applies a new current limit (amps).
     */
    public MotorControllerConfig setCurrentLimit(double currentLimit){
        this.currentLimit = currentLimit;
        return this;
    }

    /**
     * Sets whether the motor output should be inverted.
     */
    public MotorControllerConfig setInverted(boolean inverted){
        this.inverted = inverted;
        return this;
    }

    /**
     * Sets the neutral mode (brake or coast).
     */
    public MotorControllerConfig setNeutralMode(MotorNeutralMode neutralMode){
        this.neutralMode = neutralMode;
        return this;
    }

    /**
     * Updates the controller's CAN ID. Does not automatically adjust inversion.
     */
    public MotorControllerConfig setID(int id){
        this.id = id;
        return this;
    }

    /**
     * Constructs and assigns a PID controller with the provided gains.
     */
    public MotorControllerConfig setPid(double p, double i, double d) {
        return setPid(new PIDController(p, i, d));
    }

    /**
     * Assigns a fully constructed PID controller for onboard loops.
     */
    public MotorControllerConfig setPid(PIDController pid) {
        this.pid = pid;
        return this;
    }

    /** Keeps the integrated encoder on CTRE devices aligned with the selected CAN bus. */
    private void syncEncoderCanbus() {
        if (type != MotorControllerType.CTRETalonFX || encoderConfig == null) {
            return;
        }

        switch (encoderConfig.type) {
            case CTRETalonFXEncoder:
                encoderConfig.setCanbus(canbus);
            default:
                break;
        }
    }


}
