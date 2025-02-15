package ca.frc6390.athena.devices;

import java.util.function.Consumer;
import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import ca.frc6390.athena.devices.Encoder.EncoderConfig;
import ca.frc6390.athena.devices.Encoder.EncoderType;

public class MotorController {

    private final DoubleConsumer setSpeed, setVoltage, setCurrentLimit;
    private final Consumer<MotorNeutralMode> setNeutralMode;
    private final Encoder encoder;
    private boolean inverted = false;

    public enum MotorControllerType {
        CTRETalonFX(EncoderType.CTRETalonFX),
        REVSparkMaxBrushed(EncoderType.REVSparkMax),
        REVSparkMaxBrushless(EncoderType.REVSparkMax),
        REVSparkFlexBrushed(EncoderType.REVSparkFlex),
        REVSparkFlexBrushless(EncoderType.REVSparkFlex);

        private EncoderType encoder;
        MotorControllerType(EncoderType encoder){
            this.encoder = encoder;
        }


        public EncoderType getEncoder(){
            return encoder;
        }
    }

    public record MotorControllerConfig(MotorControllerType type, int id, String canbus, double currentLimit, boolean inverted, EncoderConfig encoderConfig, MotorNeutralMode neutralMode) {
        public MotorControllerConfig(MotorControllerType type, int id){
            this(type, Math.abs(id), "rio", 40, id < 0, new EncoderConfig(type.getEncoder()), MotorNeutralMode.Coast);
        }

        public MotorControllerConfig setCanbus(String canbus){
            return new MotorControllerConfig(type, id, canbus, currentLimit, inverted, encoderConfig, neutralMode);
        }

        public MotorControllerConfig withEncoderConfig(EncoderConfig encoderConfig){
            return new MotorControllerConfig(type, id, canbus, currentLimit, inverted, encoderConfig, neutralMode);
        }

        public MotorControllerConfig withCurrentLimit(double currentLimit){
            return new MotorControllerConfig(type, id, canbus, currentLimit, inverted, encoderConfig, neutralMode);
        }

        public MotorControllerConfig setInverted(boolean inverted){
            return new MotorControllerConfig(type, id, canbus, currentLimit, inverted, encoderConfig, neutralMode);
        }

        public MotorControllerConfig setNeutralMode(MotorNeutralMode neutralMode){
            return new MotorControllerConfig(type, id, canbus, currentLimit, inverted, encoderConfig, neutralMode);
        }

        public MotorControllerConfig setID(int id){
            return new MotorControllerConfig(type, id, canbus, currentLimit, inverted, encoderConfig, neutralMode);
        }

        public MotorControllerConfig setCurrentLimit(double currentLimit){
            return new MotorControllerConfig(type, id, canbus, currentLimit, inverted, encoderConfig, neutralMode);
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

    public MotorController(TalonFX controller){
        this(
            (mode) -> {
                controller.setNeutralMode(mode.asCTRE());
            },
            controller::set,
            controller::setVoltage, 
            (limit) -> {
                TalonFXConfiguration config = new TalonFXConfiguration();
                CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
                currentConfig.SupplyCurrentLimitEnable = true;
                currentConfig.SupplyCurrentLimit = limit;
                config.withCurrentLimits(currentConfig);
                controller.getConfigurator().apply(config);
            },
            new Encoder(controller)
        );
    }

    public MotorController(SparkMax controller){
        this(
            (mode) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.smartCurrentLimit(controller.configAccessor.getSmartCurrentLimit());
                config.idleMode(mode.asREV());
                controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            controller::set,
            controller::setVoltage, 
            (limit) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.smartCurrentLimit((int) limit);
                config.idleMode(controller.configAccessor.getIdleMode());
                controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            Encoder.newREVSparkMax(controller)
        );
    }

    public MotorController(SparkFlex controller){
        this(
            (mode) -> {
                SparkFlexConfig config = new SparkFlexConfig();
                config.smartCurrentLimit(controller.configAccessor.getSmartCurrentLimit());
                config.idleMode(mode.asREV());
                controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            controller::set,
            controller::setVoltage, 
            (limit) -> {
                SparkFlexConfig config = new SparkFlexConfig();
                config.smartCurrentLimit((int) limit);
                config.idleMode(controller.configAccessor.getIdleMode());
                controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            Encoder.newREVSparkFlex(controller)
        );
    }

    public MotorController(Consumer<MotorNeutralMode> setNeutralMode, DoubleConsumer setSpeed, DoubleConsumer setVoltage, DoubleConsumer setCurrentLimit, Encoder encoder) {
        this.setNeutralMode = setNeutralMode;
        this.setSpeed = setSpeed;
        this.setVoltage = setVoltage;
        this.setCurrentLimit = setCurrentLimit;
        this.encoder = encoder;
    }

    public static MotorController newREVSparkFlexBrushed(int id){
        return fromConfig(new MotorControllerConfig(MotorControllerType.REVSparkFlexBrushed, id));
    }

    public static MotorController newREVSparkFlexBrushless(int id){
        return fromConfig(new MotorControllerConfig(MotorControllerType.REVSparkFlexBrushless, id));
    }

    public static MotorController newREVSparkMaxBrushed(int id){
        return fromConfig(new MotorControllerConfig(MotorControllerType.REVSparkMaxBrushed, id));
    }

    public static MotorController newREVSparkMaxBrushless(int id){
        return fromConfig(new MotorControllerConfig(MotorControllerType.REVSparkMaxBrushless, id));
    }

    public static MotorController newCTRETalonFx(int id, String canbus){
        return fromConfig(new MotorControllerConfig(MotorControllerType.CTRETalonFX, id).setCanbus(canbus));
    }

    public static MotorController newCTRETalonFx(int id){
        return fromConfig(new MotorControllerConfig(MotorControllerType.CTRETalonFX, id));
    }


    public static MotorController fromConfig(MotorControllerConfig config) {
        switch (config.type) {
            case CTRETalonFX:
            return new MotorController(new TalonFX(config.id, config.canbus)).applyConfig(config); 
            case REVSparkMaxBrushed:
            return new MotorController(new SparkMax(config.id, MotorType.kBrushed)).applyConfig(config); 
            case REVSparkMaxBrushless:
            return new MotorController(new SparkMax(config.id, MotorType.kBrushless)).applyConfig(config); 
            case REVSparkFlexBrushed:
            return new MotorController(new SparkFlex(config.id, MotorType.kBrushed)).applyConfig(config); 
            case REVSparkFlexBrushless:
            return new MotorController(new SparkFlex(config.id, MotorType.kBrushless)).applyConfig(config); 
            default:
            return null;
        }
    }

    public MotorController applyConfig(MotorControllerConfig config) {
        withCurrentLimit(config.currentLimit);
        setInverted(config.inverted);
        setNeutralMode(config.neutralMode);
        getEncoder().applyConfig(config.encoderConfig);
        return this;
    }

    public MotorController withCurrentLimit(double limit){
        setCurrentLimit.accept(limit);
        return this;
    }

    public MotorController setInverted(boolean inverted){
        this.inverted = inverted;
        return this;
    }

    public boolean isInverted(){
        return inverted;
    }

    public MotorController setNeutralMode(MotorNeutralMode mode){
        setNeutralMode.accept(mode);
        return this;
    }

    public Encoder getEncoder() {
        return encoder;
    }

    public void setSpeed(double speed) {
        setSpeed.accept(inverted ? -speed : speed);
    }

    public void setVoltage(double voltage) {
        setVoltage.accept(inverted ? -voltage : voltage);
    }

    public void stopMotor(){
        setSpeed.accept(0);
    }


    public enum Motor {
        KRAKEN_X60_FOC(MotorControllerType.CTRETalonFX, 5800, true),
        KRAKEN_X60(MotorControllerType.CTRETalonFX,6000, false),
        FALCON_500_FOC(MotorControllerType.CTRETalonFX,6080, true),
        FALCON_500(MotorControllerType.CTRETalonFX,6380, false),
        NEO_V1(MotorControllerType.REVSparkFlexBrushless,5820, false),
        NEO_VORTEX(MotorControllerType.REVSparkFlexBrushless,6784, false);

        private final int freeSpeedRPM;
        private final boolean foc;
        private final MotorControllerType controllerType;

        Motor(MotorControllerType controllerType, int freeSpeedRPM, boolean foc) {
            this.controllerType = controllerType;
            this.freeSpeedRPM = freeSpeedRPM;
            this.foc = foc;
        }

        public int getFreeSpeedRPM() {
            return freeSpeedRPM;
        }

        public boolean isFOC() {
            return foc;
        }

        public MotorControllerType getMotorControllerType(){
            return controllerType;
        }
    }
}
