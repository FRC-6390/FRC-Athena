package ca.frc6390.athena.ctre.motor;

import java.util.Map;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import ca.frc6390.athena.ctre.CtreCanBusRegistry;
import ca.frc6390.athena.ctre.CtreConfigValidator;
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
    private static final double CONFIG_EPSILON = 1e-6;
    private static final Map<String, TalonConfigSnapshot> APPLIED_TALON_CONFIGS = new ConcurrentHashMap<>();
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
    private final StatusSignal<Boolean> faultStatorCurrentLimitSignal;
    private final StatusSignal<Boolean> faultSupplyCurrentLimitSignal;
    private final StatusSignal<?> supplyCurrentSignal;
    private final StatusSignal<?> motorVoltageSignal;
    private double cachedTemperatureCelsius = 0.0;
    private double cachedCurrentAmps = Double.NaN;
    private double cachedAppliedVoltage = Double.NaN;
    private boolean cachedConnected = true;
    private double appliedCurrentLimitAmps = Double.NaN;
    private boolean appliedInverted = false;
    private double appliedPidP = Double.NaN;
    private double appliedPidI = Double.NaN;
    private double appliedPidD = Double.NaN;
    private boolean appliedPidConfigured = false;
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
        this.faultStatorCurrentLimitSignal = controller.getFault_StatorCurrLimit(false).clone();
        this.faultSupplyCurrentLimitSignal = controller.getFault_SupplyCurrLimit(false).clone();
        this.supplyCurrentSignal = controller.getSupplyCurrent(false).clone();
        this.motorVoltageSignal = controller.getMotorVoltage(false).clone();

        deviceTemperatureSignal.setUpdateFrequency(10.0, 0.0);
        faultStatorCurrentLimitSignal.setUpdateFrequency(10.0, 0.0);
        faultSupplyCurrentLimitSignal.setUpdateFrequency(10.0, 0.0);
        supplyCurrentSignal.setUpdateFrequency(20.0, 0.0);
        motorVoltageSignal.setUpdateFrequency(20.0, 0.0);
        controller.optimizeBusUtilization(4.0, 0.0);
        appliedCurrentLimitAmps = config.currentLimit();
        appliedInverted = config.inverted();
        PIDController configuredPid = config.pid();
        if (configuredPid != null) {
            appliedPidP = configuredPid.getP();
            appliedPidI = configuredPid.getI();
            appliedPidD = configuredPid.getD();
            appliedPidConfigured = true;
        }
        cacheAppliedSnapshot();
    }

    public static CtreMotorController fromConfig(MotorControllerConfig config) {
        if (config == null || !(config.type() instanceof CtreMotorControllerType)) {
            throw new IllegalArgumentException("CTRE motor controller config required");
        }
        CtreConfigValidator.validateDeviceIdentity("TalonFX", config.id(), config.canbus());

        TalonFX talon = new TalonFX(config.id(), resolveCanBus(config.canbus()));
        TalonFXConfiguration talonConfig = new TalonFXConfiguration();
        talonConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        talonConfig.CurrentLimits.SupplyCurrentLimit = config.currentLimit();
        talonConfig.MotorOutput.NeutralMode = config.neutralMode() == MotorNeutralMode.Brake
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;
        talonConfig.MotorOutput.Inverted = config.inverted()
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        if (config.pid() != null) {
            talonConfig.Slot0.kP = config.pid().getP();
            talonConfig.Slot0.kI = config.pid().getI();
            talonConfig.Slot0.kD = config.pid().getD();
        }
        String key = deviceKey(config.id(), config.canbus());
        TalonConfigSnapshot desiredSnapshot = TalonConfigSnapshot.from(config);
        TalonConfigSnapshot appliedSnapshot = APPLIED_TALON_CONFIGS.get(key);
        if (appliedSnapshot == null || !appliedSnapshot.semanticallyEquals(desiredSnapshot)) {
            talon.getConfigurator().apply(talonConfig, 0.0);
            APPLIED_TALON_CONFIGS.put(key, desiredSnapshot);
        }
        EncoderConfig encoderCfg = config.encoderConfig();
        if (encoderCfg == null) {
            encoderCfg = EncoderConfig.create()
                    .hardware(h -> h
                            .type(CtreEncoderType.TALON_FX)
                            .id(config.id())
                            .canbus(config.canbus()))
                    .measurement(m -> m.gearRatio(1.0));
            config.encoder().config(encoderCfg);
        } else if (encoderCfg.type() == null) {
            EncoderConfig existing = encoderCfg;
            encoderCfg = EncoderConfig.create()
                    .hardware(h -> h
                            .type(CtreEncoderType.TALON_FX)
                            .id(existing.id() != 0 ? existing.id() : config.id())
                            .canbus(existing.canbus() != null ? existing.canbus() : config.canbus())
                            .inverted(existing.inverted()))
                    .measurement(m -> m
                            .gearRatio(existing.gearRatio())
                            .conversion(existing.conversion())
                            .conversionOffset(existing.conversionOffset())
                            .offset(existing.offset())
                            .discontinuity(existing.discontinuityPoint(), existing.discontinuityRange()));
            config.encoder().config(encoderCfg);
        }
        Encoder encoder = encoderCfg.type() instanceof CtreEncoderType
                ? EncoderAdapter.wrap(CtreEncoder.fromConfig(encoderCfg, talon), encoderCfg)
                : null;

        return new CtreMotorController(talon, config, encoder);
    }

    private static CANBus resolveCanBus(String canbus) {
        return CtreCanBusRegistry.resolve(canbus);
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
        if (!Double.isFinite(amps) || amps <= 0.0) {
            return;
        }
        if (nearlyEqual(appliedCurrentLimitAmps, amps)) {
            config.hardware().currentLimit(amps);
            cacheAppliedSnapshot();
            return;
        }
        config.hardware().currentLimit(amps);
        setCurrentLimit.accept(amps);
        appliedCurrentLimitAmps = amps;
        cacheAppliedSnapshot();
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
        cacheAppliedSnapshot();
    }

    @Override
    public void setPid(PIDController pid) {
        if (pid == null) {
            return;
        }
        if (appliedPidConfigured
                && nearlyEqual(appliedPidP, pid.getP())
                && nearlyEqual(appliedPidI, pid.getI())
                && nearlyEqual(appliedPidD, pid.getD())) {
            config.control().pid(pid);
            cacheAppliedSnapshot();
            return;
        }
        config.control().pid(pid);
        setPid.accept(pid);
        appliedPidP = pid.getP();
        appliedPidI = pid.getI();
        appliedPidD = pid.getD();
        appliedPidConfigured = true;
        cacheAppliedSnapshot();
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
    public double getCurrentAmps() {
        return cachedCurrentAmps;
    }

    @Override
    public double getAppliedVoltage() {
        return cachedAppliedVoltage;
    }

    @Override
    public boolean isStalled() {
        faultStatorCurrentLimitSignal.refresh(false);
        faultSupplyCurrentLimitSignal.refresh(false);
        return Boolean.TRUE.equals(faultStatorCurrentLimitSignal.getValue())
                || Boolean.TRUE.equals(faultSupplyCurrentLimitSignal.getValue());
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
        if (appliedInverted == inverted) {
            config.hardware().inverted(inverted);
            cacheAppliedSnapshot();
            return;
        }
        config.hardware().inverted(inverted);
        MotorOutputConfigs output = new MotorOutputConfigs();
        output.Inverted = inverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;
        output.NeutralMode = config.neutralMode() == MotorNeutralMode.Brake
                ? NeutralModeValue.Brake
                : NeutralModeValue.Coast;
        controller.getConfigurator().apply(output, 0.0);
        appliedInverted = inverted;
        cacheAppliedSnapshot();
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
            faultStatorCurrentLimitSignal.refresh(false);
            faultSupplyCurrentLimitSignal.refresh(false);
            supplyCurrentSignal.refresh(false);
            motorVoltageSignal.refresh(false);
            cachedTemperatureCelsius = deviceTemperatureSignal.getValueAsDouble();
            cachedCurrentAmps = supplyCurrentSignal.getValueAsDouble();
            cachedAppliedVoltage = motorVoltageSignal.getValueAsDouble();
        }
        if (updateCycles >= 1_000_000) {
            updateCycles = 0;
        }
        cachedConnected = controller.isConnected();
    }

    private static boolean nearlyEqual(double first, double second) {
        if (Double.doubleToLongBits(first) == Double.doubleToLongBits(second)) {
            return true;
        }
        return Double.isFinite(first) && Double.isFinite(second) && Math.abs(first - second) <= CONFIG_EPSILON;
    }

    private void cacheAppliedSnapshot() {
        APPLIED_TALON_CONFIGS.put(deviceKey(config.id(), config.canbus()), TalonConfigSnapshot.from(config));
    }

    private static String deviceKey(int id, String canbus) {
        String resolvedCanbus = canbus != null ? canbus : "";
        return resolvedCanbus + "\\" + id;
    }

    private record TalonConfigSnapshot(
            double currentLimitAmps,
            boolean inverted,
            MotorNeutralMode neutralMode,
            boolean hasPid,
            double pidP,
            double pidI,
            double pidD) {

        static TalonConfigSnapshot from(MotorControllerConfig config) {
            PIDController pid = config != null ? config.pid() : null;
            return new TalonConfigSnapshot(
                    config != null ? config.currentLimit() : Double.NaN,
                    config != null && config.inverted(),
                    config != null ? config.neutralMode() : MotorNeutralMode.Coast,
                    pid != null,
                    pid != null ? pid.getP() : Double.NaN,
                    pid != null ? pid.getI() : Double.NaN,
                    pid != null ? pid.getD() : Double.NaN);
        }

        boolean semanticallyEquals(TalonConfigSnapshot other) {
            if (other == null) {
                return false;
            }
            if (!nearlyEqual(currentLimitAmps, other.currentLimitAmps)) {
                return false;
            }
            if (inverted != other.inverted || neutralMode != other.neutralMode || hasPid != other.hasPid) {
                return false;
            }
            if (!hasPid) {
                return true;
            }
            return nearlyEqual(pidP, other.pidP)
                    && nearlyEqual(pidI, other.pidI)
                    && nearlyEqual(pidD, other.pidD);
        }
    }
}
