package ca.frc6390.athena.rev.motor;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import com.revrobotics.PersistMode;
import com.revrobotics.REVLibError;
import com.revrobotics.ResetMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import ca.frc6390.athena.rev.encoder.RevEncoder;
import ca.frc6390.athena.rev.encoder.RevEncoderType;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderAdapter;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * REV motor controller wrapper for the new vendordep system.
 */
public class RevMotorController implements MotorController {
    private static final ResetMode INITIAL_RESET_MODE = ResetMode.kResetSafeParameters;
    private static final ResetMode UPDATE_RESET_MODE = ResetMode.kNoResetSafeParameters;
    private static final PersistMode RUNTIME_PERSIST_MODE = PersistMode.kNoPersistParameters;
    private static final int CAN_TIMEOUT_MS = 50;

    private final MotorControllerConfig config;
    private final Encoder encoder;
    private final Consumer<Double> setSpeed;
    private final Consumer<Double> setVoltage;
    private final Consumer<Double> setCurrentLimit;
    private final Consumer<Double> setPosition;
    private final Consumer<Double> setVelocity;
    private final Consumer<MotorNeutralMode> setNeutralMode;
    private final Consumer<PIDController> setPid;
    private final Consumer<Boolean> setInverted;
    private final BooleanSupplier getConnected;
    private final java.util.function.DoubleSupplier getTemperature;
    private final BooleanSupplier getInverted;

    public RevMotorController(MotorControllerConfig config,
                              Encoder encoder,
                              Consumer<Double> setSpeed,
                              Consumer<Double> setVoltage,
                              Consumer<Double> setCurrentLimit,
                              Consumer<Double> setPosition,
                              Consumer<Double> setVelocity,
                              Consumer<MotorNeutralMode> setNeutralMode,
                              Consumer<PIDController> setPid,
                              Consumer<Boolean> setInverted,
                              BooleanSupplier getConnected,
                              java.util.function.DoubleSupplier getTemperature,
                              BooleanSupplier getInverted) {
        this.config = config;
        this.encoder = encoder;
        this.setSpeed = setSpeed;
        this.setVoltage = setVoltage;
        this.setCurrentLimit = setCurrentLimit;
        this.setPosition = setPosition;
        this.setVelocity = setVelocity;
        this.setNeutralMode = setNeutralMode;
        this.setPid = setPid;
        this.setInverted = setInverted;
        this.getConnected = getConnected;
        this.getTemperature = getTemperature;
        this.getInverted = getInverted;
    }

    public static RevMotorController fromConfig(MotorControllerConfig config) {
        if (config == null || !(config.type() instanceof RevMotorControllerType)) {
            throw new IllegalArgumentException("REV motor controller config required");
        }

        RevMotorControllerType type = (RevMotorControllerType) config.type();
        return switch (type) {
            case SPARK_MAX_BRUSHED -> createSparkMax(config, MotorType.kBrushed);
            case SPARK_MAX_BRUSHLESS -> createSparkMax(config, MotorType.kBrushless);
            case SPARK_FLEX_BRUSHED -> createSparkFlex(config, MotorType.kBrushed);
            case SPARK_FLEX_BRUSHLESS -> createSparkFlex(config, MotorType.kBrushless);
        };
    }

    private static RevMotorController createSparkMax(MotorControllerConfig config, MotorType motorType) {
        SparkMax controller = new SparkMax(config.id(), motorType);
        setCanTimeout(controller, config.id());
        SparkMaxConfig cfg = new SparkMaxConfig();
        cfg.idleMode(config.neutralMode() == MotorNeutralMode.Brake
                ? com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake
                : com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
        cfg.smartCurrentLimit((int) config.currentLimit());
        if (config.pid() != null) {
            cfg.closedLoop.pid(config.pid().getP(), config.pid().getI(), config.pid().getD());
        }
        applyConfig(controller, cfg, INITIAL_RESET_MODE, RUNTIME_PERSIST_MODE, config.id(), "initial");

        EncoderConfig encoderCfg = config.encoderConfig();
        Encoder encoder = null;
        if (encoderCfg != null && encoderCfg.type() instanceof RevEncoderType revType
                && revType == RevEncoderType.SPARK_MAX
                && (encoderCfg.id() == 0 || encoderCfg.id() == config.id())) {
            encoder = EncoderAdapter.wrap(
                    new RevEncoder(controller.getEncoder(), safeAbsoluteEncoder(controller), encoderCfg),
                    encoderCfg);
        } else if (encoderCfg != null && encoderCfg.type() instanceof RevEncoderType) {
            encoder = EncoderAdapter.wrap(RevEncoder.fromConfig(encoderCfg), encoderCfg);
        } else {
            EncoderConfig fallback = EncoderConfig.create()
                    .hardware(h -> h
                            .type(RevEncoderType.SPARK_MAX)
                            .id(config.id())
                            .canbus(config.canbus())
                            .inverted(config.encoderConfig() != null && config.encoderConfig().inverted()))
                    .measurement(m -> m
                            .gearRatio(config.encoderConfig() != null ? config.encoderConfig().gearRatio() : 1.0)
                            .conversion(config.encoderConfig() != null ? config.encoderConfig().conversion() : 1.0)
                            .conversionOffset(config.encoderConfig() != null
                                    ? config.encoderConfig().conversionOffset()
                                    : 0.0)
                            .offset(config.encoderConfig() != null ? config.encoderConfig().offset() : 0.0)
                            .discontinuity(
                                    config.encoderConfig() != null
                                            ? config.encoderConfig().discontinuityPoint()
                                            : Double.NaN,
                                    config.encoderConfig() != null
                                            ? config.encoderConfig().discontinuityRange()
                                            : Double.NaN));
            encoder = EncoderAdapter.wrap(
                    new RevEncoder(controller.getEncoder(), safeAbsoluteEncoder(controller), fallback),
                    fallback);
            config.encoder().config(fallback);
        }

        return new RevMotorController(
                config,
                encoder,
                controller::set,
                controller::setVoltage,
                limit -> {
                    SparkMaxConfig update = new SparkMaxConfig();
                    update.smartCurrentLimit((int) Math.round(limit));
                    applyConfig(controller, update, UPDATE_RESET_MODE, RUNTIME_PERSIST_MODE, config.id(), "currentLimit");
                },
                val -> controller.getClosedLoopController().setSetpoint(val, ControlType.kPosition),
                val -> controller.getClosedLoopController().setSetpoint(val * 60.0, ControlType.kVelocity),
                mode -> {
                    SparkMaxConfig update = new SparkMaxConfig();
                    update.idleMode(mode == MotorNeutralMode.Brake
                            ? com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake
                            : com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
                    applyConfig(controller, update, UPDATE_RESET_MODE, RUNTIME_PERSIST_MODE, config.id(), "neutralMode");
                },
                pid -> {
                    SparkMaxConfig update = new SparkMaxConfig();
                    update.closedLoop.pid(pid.getP(), pid.getI(), pid.getD());
                    applyConfig(controller, update, UPDATE_RESET_MODE, RUNTIME_PERSIST_MODE, config.id(), "pid");
                },
                controller::setInverted,
                () -> !controller.hasActiveFault(),
                controller::getMotorTemperature,
                controller::getInverted);
    }

    private static RevMotorController createSparkFlex(MotorControllerConfig config, MotorType motorType) {
        SparkFlex controller = new SparkFlex(config.id(), motorType);
        setCanTimeout(controller, config.id());
        SparkFlexConfig cfg = new SparkFlexConfig();
        cfg.idleMode(config.neutralMode() == MotorNeutralMode.Brake
                ? com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake
                : com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
        cfg.smartCurrentLimit((int) config.currentLimit());
        if (config.pid() != null) {
            cfg.closedLoop.pid(config.pid().getP(), config.pid().getI(), config.pid().getD());
        }
        applyConfig(controller, cfg, INITIAL_RESET_MODE, RUNTIME_PERSIST_MODE, config.id(), "initial");

        EncoderConfig encoderCfg = config.encoderConfig();
        Encoder encoder = null;
        if (encoderCfg != null && encoderCfg.type() instanceof RevEncoderType revType
                && revType == RevEncoderType.SPARK_FLEX
                && (encoderCfg.id() == 0 || encoderCfg.id() == config.id())) {
            encoder = EncoderAdapter.wrap(
                    new RevEncoder(controller.getEncoder(), safeAbsoluteEncoder(controller), encoderCfg),
                    encoderCfg);
        } else if (encoderCfg != null && encoderCfg.type() instanceof RevEncoderType) {
            encoder = EncoderAdapter.wrap(RevEncoder.fromConfig(encoderCfg), encoderCfg);
        } else {
            EncoderConfig fallback = EncoderConfig.create()
                    .hardware(h -> h
                            .type(RevEncoderType.SPARK_FLEX)
                            .id(config.id())
                            .canbus(config.canbus())
                            .inverted(config.encoderConfig() != null && config.encoderConfig().inverted()))
                    .measurement(m -> m
                            .gearRatio(config.encoderConfig() != null ? config.encoderConfig().gearRatio() : 1.0)
                            .conversion(config.encoderConfig() != null ? config.encoderConfig().conversion() : 1.0)
                            .conversionOffset(config.encoderConfig() != null
                                    ? config.encoderConfig().conversionOffset()
                                    : 0.0)
                            .offset(config.encoderConfig() != null ? config.encoderConfig().offset() : 0.0)
                            .discontinuity(
                                    config.encoderConfig() != null
                                            ? config.encoderConfig().discontinuityPoint()
                                            : Double.NaN,
                                    config.encoderConfig() != null
                                            ? config.encoderConfig().discontinuityRange()
                                            : Double.NaN));
            encoder = EncoderAdapter.wrap(
                    new RevEncoder(controller.getEncoder(), safeAbsoluteEncoder(controller), fallback),
                    fallback);
            config.encoder().config(fallback);
        }

        return new RevMotorController(
                config,
                encoder,
                controller::set,
                controller::setVoltage,
                limit -> {
                    SparkFlexConfig update = new SparkFlexConfig();
                    update.smartCurrentLimit((int) Math.round(limit));
                    applyConfig(controller, update, UPDATE_RESET_MODE, RUNTIME_PERSIST_MODE, config.id(), "currentLimit");
                },
                val -> controller.getClosedLoopController().setSetpoint(val, ControlType.kPosition),
                val -> controller.getClosedLoopController().setSetpoint(val * 60.0, ControlType.kVelocity),
                mode -> {
                    SparkFlexConfig update = new SparkFlexConfig();
                    update.idleMode(mode == MotorNeutralMode.Brake
                            ? com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kBrake
                            : com.revrobotics.spark.config.SparkBaseConfig.IdleMode.kCoast);
                    applyConfig(controller, update, UPDATE_RESET_MODE, RUNTIME_PERSIST_MODE, config.id(), "neutralMode");
                },
                pid -> {
                    SparkFlexConfig update = new SparkFlexConfig();
                    update.closedLoop.pid(pid.getP(), pid.getI(), pid.getD());
                    applyConfig(controller, update, UPDATE_RESET_MODE, RUNTIME_PERSIST_MODE, config.id(), "pid");
                },
                controller::setInverted,
                () -> !controller.hasActiveFault(),
                controller::getMotorTemperature,
                controller::getInverted);
    }

    @Override
    public int getId() {
        return config.id();
    }

    @Override
    public String getCanbus() {
        return config.canbus();
    }

    @Override
    public MotorControllerType getType() {
        return config.type();
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
        config.hardware().currentLimit(amps);
        setCurrentLimit.accept(amps);
    }

    @Override
    public void setPosition(double rotations) {
        setPosition.accept(rotations);
    }

    @Override
    public void setVelocity(double rotationsPerSecond) {
        setVelocity.accept(rotationsPerSecond);
    }

    @Override
    public void setNeutralMode(MotorNeutralMode mode) {
        config.hardware().neutralMode(mode);
        setNeutralMode.accept(mode);
    }

    @Override
    public void setPid(PIDController pid) {
        config.control().pid(pid);
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

    @Override
    public boolean isInverted() {
        return config.inverted();
    }

    @Override
    public void setInverted(boolean inverted) {
        config.hardware().inverted(inverted);
        setInverted.accept(inverted);
    }

    @Override
    public double getCurrentLimit() {
        return config.currentLimit();
    }

    @Override
    public MotorNeutralMode getNeutralMode() {
        return config.neutralMode();
    }

    @Override
    public MotorControllerConfig getConfig() {
        return config;
    }

    private static AbsoluteEncoder safeAbsoluteEncoder(SparkMax controller) {
        try {
            return controller.getAbsoluteEncoder();
        } catch (RuntimeException ex) {
            return null;
        }
    }

    private static AbsoluteEncoder safeAbsoluteEncoder(SparkFlex controller) {
        try {
            return controller.getAbsoluteEncoder();
        } catch (RuntimeException ex) {
            return null;
        }
    }

    private static void setCanTimeout(SparkBase controller, int deviceId) {
        REVLibError status = controller.setCANTimeout(CAN_TIMEOUT_MS);
        if (status != REVLibError.kOk) {
            DriverStation.reportWarning(
                    "[Athena][REV] Failed to set CAN timeout for device " + deviceId + ": " + status,
                    false);
        }
    }

    private static void applyConfig(
            SparkBase controller,
            SparkBaseConfig config,
            ResetMode resetMode,
            PersistMode persistMode,
            int deviceId,
            String phase) {
        REVLibError status = controller.configure(config, resetMode, persistMode);
        if (status == REVLibError.kOk) {
            return;
        }
        DriverStation.reportWarning(
                "[Athena][REV] Configure " + phase + " failed for device " + deviceId + ": " + status,
                false);
    }
}
