package ca.frc6390.athena.ctre;

import java.util.function.Consumer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.frc6390.athena.hardware.Encoder;
import ca.frc6390.athena.hardware.MotorController;
import ca.frc6390.athena.hardware.MotorControllerConfig;
import ca.frc6390.athena.hardware.MotorControllerType;
import ca.frc6390.athena.hardware.MotorNeutralMode;
import edu.wpi.first.math.controller.PIDController;

/**
 * CTRE motor controller wrapper for the new vendordep system.
 */
public class CtreMotorController implements MotorController {
    private final TalonFX controller;
    private final MotorControllerConfig config;
    private final Encoder encoder;
    private final Consumer<Double> setSpeed;
    private final Consumer<Double> setVoltage;
    private final Consumer<Double> setCurrentLimit;
    private final Consumer<Double> setPosition;
    private final Consumer<MotorNeutralMode> setNeutralMode;
    private final Consumer<PIDController> setPid;

    public CtreMotorController(TalonFX controller, MotorControllerConfig config, Encoder encoder) {
        this.controller = controller;
        this.config = config;
        this.encoder = encoder;
        this.setSpeed = controller::set;
        this.setVoltage = controller::setVoltage;
        this.setCurrentLimit = limit -> {
            TalonFXConfiguration cfg = new TalonFXConfiguration();
            CurrentLimitsConfigs current = new CurrentLimitsConfigs();
            current.SupplyCurrentLimitEnable = true;
            current.SupplyCurrentLimit = limit;
            cfg.withCurrentLimits(current);
            controller.getConfigurator().apply(cfg);
        };
        this.setPosition = val -> controller.setControl(new PositionDutyCycle(val).withSlot(0));
        this.setNeutralMode = mode -> controller.setNeutralMode(mode == MotorNeutralMode.Brake
                ? com.ctre.phoenix6.signals.NeutralModeValue.Brake
                : com.ctre.phoenix6.signals.NeutralModeValue.Coast);
        this.setPid = pid -> {
            Slot0Configs slot0 = new Slot0Configs();
            slot0.kP = pid.getP();
            slot0.kI = pid.getI();
            slot0.kD = pid.getD();
            controller.getConfigurator().apply(slot0);
        };
    }

    public static CtreMotorController fromConfig(MotorControllerConfig config) {
        if (config == null || !(config.type instanceof CtreMotorControllerType)) {
            throw new IllegalArgumentException("CTRE motor controller config required");
        }

        TalonFX talon = new TalonFX(config.id, config.canbus);
        Encoder encoder = null;
        if (config.encoderConfig != null && config.encoderConfig.type instanceof CtreEncoderType) {
            encoder = CtreEncoder.fromConfig(config.encoderConfig);
        }
        CtreMotorController controller = new CtreMotorController(talon, config, encoder);
        controller.setCurrentLimit(config.currentLimit);
        controller.setNeutralMode(config.neutralMode);
        if (config.pid != null) {
            controller.setPid(config.pid);
        }
        return controller;
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
        return controller.isConnected();
    }

    @Override
    public double getTemperatureCelsius() {
        return controller.getDeviceTemp(true).getValueAsDouble();
    }

    @Override
    public Encoder getEncoder() {
        return encoder;
    }
}
