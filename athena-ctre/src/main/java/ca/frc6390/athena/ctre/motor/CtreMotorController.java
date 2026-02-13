package ca.frc6390.athena.ctre.motor;

import java.util.function.Consumer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.frc6390.athena.ctre.CtreCanBusRegistry;
import ca.frc6390.athena.ctre.encoder.CtreEncoder;
import ca.frc6390.athena.ctre.encoder.CtreEncoderType;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderAdapter;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import edu.wpi.first.math.controller.PIDController;

/**
 * CTRE motor controller wrapper for the new vendordep system.
 */
public class CtreMotorController implements MotorController {
    private static final int TEMPERATURE_REFRESH_INTERVAL_CYCLES = 25;
    private final TalonFX controller;
    private final MotorControllerConfig config;
    private final Encoder encoder;
    private final Consumer<Double> setSpeed;
    private final Consumer<Double> setVoltage;
    private final Consumer<Double> setCurrentLimit;
    private final Consumer<Double> setPosition;
    private final Consumer<Double> setVelocity;
    private final Consumer<MotorNeutralMode> setNeutralMode;
    private final Consumer<PIDController> setPid;
    private final StatusSignal<?> deviceTemperatureSignal;
    private double cachedTemperatureCelsius = 0.0;
    private boolean cachedConnected = true;
    private int updateCycles = 0;

    public CtreMotorController(TalonFX controller, MotorControllerConfig config, Encoder encoder) {
        this.controller = controller;
        this.config = config;
        this.encoder = encoder;
        this.deviceTemperatureSignal = controller.getDeviceTemp(false).clone();
        this.setSpeed = controller::set;
        this.setVoltage = controller::setVoltage;
        this.setCurrentLimit = limit -> {
            CurrentLimitsConfigs current = new CurrentLimitsConfigs();
            current.SupplyCurrentLimitEnable = true;
            current.SupplyCurrentLimit = limit;
            controller.getConfigurator().apply(current, 0.0);
        };
        this.setPosition = val -> controller.setControl(new PositionDutyCycle(val).withSlot(0));
        this.setVelocity = val -> controller.setControl(new VelocityDutyCycle(val).withSlot(0));
        this.setNeutralMode = mode -> controller.setNeutralMode(mode == MotorNeutralMode.Brake
                ? com.ctre.phoenix6.signals.NeutralModeValue.Brake
                : com.ctre.phoenix6.signals.NeutralModeValue.Coast);
        this.setPid = pid -> {
            Slot0Configs slot0 = new Slot0Configs();
            slot0.kP = pid.getP();
            slot0.kI = pid.getI();
            slot0.kD = pid.getD();
            controller.getConfigurator().apply(slot0, 0.0);
        };

        deviceTemperatureSignal.setUpdateFrequency(10.0, 0.0);
        controller.optimizeBusUtilization(4.0, 0.0);
    }

    public static CtreMotorController fromConfig(MotorControllerConfig config) {
        if (config == null || !(config.type instanceof CtreMotorControllerType)) {
            throw new IllegalArgumentException("CTRE motor controller config required");
        }

        TalonFX talon = new TalonFX(config.id, resolveCanBus(config.canbus));
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLimit = config.currentLimit;
        talonConfig.MotorOutput.NeutralMode = config.neutralMode == MotorNeutralMode.Brake
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;
        if (config.pid != null) {
            talonConfig.Slot0.kP = config.pid.getP();
            talonConfig.Slot0.kI = config.pid.getI();
            talonConfig.Slot0.kD = config.pid.getD();
        }
        talon.getConfigurator().apply(talonConfig, 0.0);
        EncoderConfig encoderCfg = config.encoderConfig;
        if (encoderCfg == null) {
            encoderCfg = new EncoderConfig()
                    .setType(CtreEncoderType.TALON_FX)
                    .setId(config.id)
                    .setCanbus(config.canbus)
                    .setGearRatio(1.0);
        } else if (encoderCfg.type == null) {
            encoderCfg = new EncoderConfig()
                    .setType(CtreEncoderType.TALON_FX)
                    .setId(encoderCfg.id != 0 ? encoderCfg.id : config.id)
                    .setCanbus(encoderCfg.canbus != null ? encoderCfg.canbus : config.canbus)
                    .setGearRatio(encoderCfg.gearRatio)
                    .setConversion(encoderCfg.conversion)
                    .setConversionOffset(encoderCfg.conversionOffset)
                    .setOffset(encoderCfg.offset)
                    .setDiscontinuity(encoderCfg.discontinuityPoint, encoderCfg.discontinuityRange)
                    .setInverted(encoderCfg.inverted);
        }
        Encoder encoder = encoderCfg.type instanceof CtreEncoderType
                ? EncoderAdapter.wrap(CtreEncoder.fromConfig(encoderCfg, talon), encoderCfg)
                : null;

        return new CtreMotorController(talon, config, encoder);
    }

    private static CANBus resolveCanBus(String canbus) {
        return CtreCanBusRegistry.resolve(canbus);
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
        config.currentLimit = amps;
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
        config.neutralMode = mode;
        setNeutralMode.accept(mode);
    }

    @Override
    public void setPid(PIDController pid) {
        config.pid = pid;
        setPid.accept(pid);
    }

    @Override
    public boolean isConnected() {
        return cachedConnected;
    }

    @Override
    public boolean isConnected(boolean poll) {
        if (poll) {
            cachedConnected = controller.isConnected();
        }
        return cachedConnected;
    }

    @Override
    public double getTemperatureCelsius() {
        return cachedTemperatureCelsius;
    }

    @Override
    public double getTemperatureCelsius(boolean poll) {
        if (poll) {
            deviceTemperatureSignal.refresh(false);
            cachedTemperatureCelsius = deviceTemperatureSignal.getValueAsDouble();
        }
        return cachedTemperatureCelsius;
    }

    @Override
    public Encoder getEncoder() {
        return encoder;
    }

    @Override
    public boolean isInverted() {
        return config.inverted;
    }

    @Override
    public void setInverted(boolean inverted) {
        config.inverted = inverted;
    }

    @Override
    public double getCurrentLimit() {
        return config.currentLimit;
    }

    @Override
    public MotorNeutralMode getNeutralMode() {
        return config.neutralMode;
    }

    @Override
    public MotorControllerConfig getConfig() {
        return config;
    }

    @Override
    public String getName() {
        return getCanbus() + "\\" + getId() + "\\" + getType().getKey();
    }

    @Override
    public void stopMotor() {
        controller.set(0);
    }

    @Override
    public void update() {
        if (encoder != null) {
            encoder.update();
        }
        updateCycles++;
        if (updateCycles == 1 || updateCycles % TEMPERATURE_REFRESH_INTERVAL_CYCLES == 0) {
            deviceTemperatureSignal.refresh(false);
            cachedTemperatureCelsius = deviceTemperatureSignal.getValueAsDouble();
        }
        if (updateCycles >= 1_000_000) {
            updateCycles = 0;
        }
        cachedConnected = controller.isConnected();
    }
}
