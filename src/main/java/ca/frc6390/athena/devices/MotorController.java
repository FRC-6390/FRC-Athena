package ca.frc6390.athena.devices;

import java.util.function.DoubleConsumer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import ca.frc6390.athena.devices.Encoder.EncoderConfig;

public class MotorController {

    private final DoubleConsumer setSpeed, setVoltage, setCurrentLimit;
    private final Encoder relativeEncoder, absoluteEncoder;

    public enum MotorControllerType {
        CTRETalonFX,
        REVSparkMaxBrushed,
        REVSparkMaxBrushless,
        REVSparkFlexBrushed,
        REVSparkFlexBrushless,
    }

    public record MotorControllerConfig(MotorControllerType type, int id, String canbus, double currentLimit, EncoderConfig encoderConfig) {
        public MotorControllerConfig(MotorControllerType type, int id){
            this(type, id, "rio", 40, new EncoderConfig());
        }

        public MotorControllerConfig withCanbus(String canbus){
            return new MotorControllerConfig(type, id, canbus, currentLimit, encoderConfig);
        }

        public MotorControllerConfig withEncoderConfig(EncoderConfig encoderConfig){
            return new MotorControllerConfig(type, id, canbus, currentLimit, encoderConfig);
        }

        public MotorControllerConfig withCurrentLimit(double currentLimit){
            return new MotorControllerConfig(type, id, canbus, currentLimit, encoderConfig);
        }
    }

    public MotorController(TalonFX controller){
        this(
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
            new Encoder(controller),
            new Encoder(controller, true)
        );
    }

    public MotorController(SparkMax controller){
        this(
            controller::set,
            controller::setVoltage, 
            (limit) -> {
                SparkMaxConfig config = new SparkMaxConfig();
                config.smartCurrentLimit((int) limit);
                controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            new Encoder(controller.getEncoder()),
            new Encoder(controller.getAbsoluteEncoder())
        );
    }

    public MotorController(SparkFlex controller){
        this(
            controller::set,
            controller::setVoltage, 
            (limit) -> {
                SparkFlexConfig config = new SparkFlexConfig();
                config.smartCurrentLimit((int) limit);
                controller.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
            },
            new Encoder(controller.getEncoder()),
            new Encoder(controller.getAbsoluteEncoder())
        );
    }

    public MotorController(DoubleConsumer setSpeed, DoubleConsumer setVoltage, DoubleConsumer setCurrentLimit, Encoder relativeEncoder, Encoder absoluteEncoder) {
        this.setSpeed = setSpeed;
        this.setVoltage = setVoltage;
        this.setCurrentLimit = setCurrentLimit;
        this.relativeEncoder = relativeEncoder;
        this.absoluteEncoder = absoluteEncoder;
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
        return fromConfig(new MotorControllerConfig(MotorControllerType.CTRETalonFX, id).withCanbus(canbus));
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
        getAbsoluteEncoder().applyConfig(config.encoderConfig);
        getRelativeEncoder().applyConfig(config.encoderConfig);
        return this;
    }

    public MotorController withCurrentLimit(double limit){
        setCurrentLimit.accept(limit);
        return this;
    }

    public Encoder getRelativeEncoder() {
        return relativeEncoder;
    }

    public Encoder getAbsoluteEncoder() {
        return absoluteEncoder;
    }

    public void setSpeed(double speed) {
        setSpeed.accept(speed);
    }

    public void setVoltage(double voltage) {
        setVoltage.accept(voltage);
    }

    public void stopMotor(){
        setSpeed.accept(0);
    }
}
