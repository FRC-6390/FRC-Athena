package ca.frc6390.athena.rev.motor;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import ca.frc6390.athena.rev.encoder.RevEncoder;
import ca.frc6390.athena.rev.encoder.RevEncoderType;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import edu.wpi.first.math.controller.PIDController;

/**
 * REV motor controller wrapper for the new vendordep system.
 */
public class RevMotorController implements MotorController {
    private final MotorControllerConfig config;
    private final Encoder encoder;
    private final Consumer<Double> setSpeed;
    private final Consumer<Double> setVoltage;
    private final Consumer<Double> setCurrentLimit;
    private final Consumer<Double> setPosition;
    private final Consumer<MotorNeutralMode> setNeutralMode;
    private final Consumer<PIDController> setPid;
    private final BooleanSupplier getConnected;
    private final java.util.function.DoubleSupplier getTemperature;

    public RevMotorController(MotorControllerConfig config,
                              Encoder encoder,
                              Consumer<Double> setSpeed,
                              Consumer<Double> setVoltage,
                              Consumer<Double> setCurrentLimit,
                              Consumer<Double> setPosition,
                              Consumer<MotorNeutralMode> setNeutralMode,
                              Consumer<PIDController> setPid,
                              BooleanSupplier getConnected,
                              java.util.function.DoubleSupplier getTemperature) {
        this.config = config;
        this.encoder = encoder;
        this.setSpeed = setSpeed;
        this.setVoltage = setVoltage;
        this.setCurrentLimit = setCurrentLimit;
        this.setPosition = setPosition;
        this.setNeutralMode = setNeutralMode;
        this.setPid = setPid;
        this.getConnected = getConnected;
        this.getTemperature = getTemperature;
    }

    public static RevMotorController fromConfig(MotorControllerConfig config) {
        if (config == null || !(config.type instanceof RevMotorControllerType)) {
            throw new IllegalArgumentException("REV motor controller config required");
        }

        RevMotorControllerType type = (RevMotorControllerType) config.type;
        return switch (type) {
            case SPARK_MAX_BRUSHED -> createSparkMax(config, MotorType.kBrushed);
            case SPARK_MAX_BRUSHLESS -> createSparkMax(config, MotorType.kBrushless);
            case SPARK_FLEX_BRUSHED -> createSparkFlex(config, MotorType.kBrushed);
            case SPARK_FLEX_BRUSHLESS -> createSparkFlex(config, MotorType.kBrushless);
        };
    }

    private static RevMotorController createSparkMax(MotorControllerConfig config, MotorType motorType) {
        SparkMax controller = new SparkMax(config.id, motorType);
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(config.neutralMode == MotorNeutralMode.Brake
                ? com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake
                : com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
        cfg.smartCurrentLimit((int) config.currentLimit);
        controller.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        Encoder encoder = null;
        if (config.encoderConfig != null && config.encoderConfig.type instanceof RevEncoderType) {
            encoder = RevEncoder.fromConfig(config.encoderConfig);
        } else if (encoder == null) {
            EncoderConfig encoderCfg = new EncoderConfig()
                    .setType(RevEncoderType.SPARK_MAX)
                    .setId(config.id)
                    .setCanbus(config.canbus)
                    .setGearRatio(config.encoderConfig != null ? config.encoderConfig.gearRatio : 1.0)
                    .setConversion(config.encoderConfig != null ? config.encoderConfig.conversion : 1.0)
                    .setOffset(config.encoderConfig != null ? config.encoderConfig.offset : 0.0)
                    .setInverted(config.encoderConfig != null && config.encoderConfig.inverted);
            encoder = RevEncoder.fromConfig(encoderCfg);
        }

        return new RevMotorController(
                config,
                encoder,
                controller::set,
                controller::setVoltage,
                limit -> {
                    SparkMaxConfig update = new SparkMaxConfig();
                    update.smartCurrentLimit((int) Math.round(limit));
                    controller.configure(update, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                },
                val -> controller.getClosedLoopController().setSetpoint(val, ControlType.kPosition),
                mode -> {
                    SparkMaxConfig update = new SparkMaxConfig();
                    update.idleMode(mode == MotorNeutralMode.Brake
                            ? com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake
                            : com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
                    controller.configure(update, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                },
                pid -> {
                    SparkMaxConfig update = new SparkMaxConfig();
                    update.closedLoop.pid(pid.getP(), pid.getI(), pid.getD());
                    controller.configure(update, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                },
                () -> !controller.hasActiveFault(),
                controller::getMotorTemperature);
    }

    private static RevMotorController createSparkFlex(MotorControllerConfig config, MotorType motorType) {
        SparkFlex controller = new SparkFlex(config.id, motorType);
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.idleMode(config.neutralMode == MotorNeutralMode.Brake
                ? com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake
                : com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
        cfg.smartCurrentLimit((int) config.currentLimit);
        controller.configure(cfg, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        Encoder encoder = null;
        if (config.encoderConfig != null && config.encoderConfig.type instanceof RevEncoderType) {
            encoder = RevEncoder.fromConfig(config.encoderConfig);
        }

        return new RevMotorController(
                config,
                encoder,
                controller::set,
                controller::setVoltage,
                limit -> {
                    SparkFlexConfig update = new SparkFlexConfig();
                    update.smartCurrentLimit((int) Math.round(limit));
                    controller.configure(update, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                },
                val -> controller.getClosedLoopController().setSetpoint(val, ControlType.kPosition),
                mode -> {
                    SparkFlexConfig update = new SparkFlexConfig();
                    update.idleMode(mode == MotorNeutralMode.Brake
                            ? com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake
                            : com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
                    controller.configure(update, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                },
                pid -> {
                    SparkFlexConfig update = new SparkFlexConfig();
                    update.closedLoop.pid(pid.getP(), pid.getI(), pid.getD());
                    controller.configure(update, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
                },
                () -> !controller.hasActiveFault(),
                controller::getMotorTemperature);
    }

    @Override
    public int getId() {
        return config.id;
    }

    @Override
    public String getCanbus() {
        return config.canbus;
    }

    @Override
    public MotorControllerType getType() {
        return config.type;
    }

    @Override
    public void setSpeed(double percent) {
        setSpeed.accept(percent);
    }

    @Override
    public void setVoltage(double volts) {
        setVoltage.accept(volts);
    }

    @Override
    public void setCurrentLimit(double amps) {
        setCurrentLimit.accept(amps);
    }

    @Override
    public void setPosition(double rotations) {
        setPosition.accept(rotations);
    }

    @Override
    public void setNeutralMode(MotorNeutralMode mode) {
        setNeutralMode.accept(mode);
    }

    @Override
    public void setPid(PIDController pid) {
        setPid.accept(pid);
    }

    @Override
    public boolean isConnected() {
        return getConnected.getAsBoolean();
    }

    @Override
    public double getTemperatureCelsius() {
        return getTemperature.getAsDouble();
    }

    @Override
    public Encoder getEncoder() {
        return encoder;
    }
}
