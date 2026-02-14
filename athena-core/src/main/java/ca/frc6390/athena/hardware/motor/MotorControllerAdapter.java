package ca.frc6390.athena.hardware.motor;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderAdapter;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import edu.wpi.first.math.controller.PIDController;

/**
 * Applies shared motor configuration (inversion, limits) on top of vendor-specific controllers.
 */
public class MotorControllerAdapter implements MotorController {
    private static final int CONNECTION_POLL_INTERVAL_CYCLES = 10;
    private static final int TEMPERATURE_POLL_INTERVAL_CYCLES = 25;
    private final MotorController raw;
    private final MotorControllerConfig config;
    private Encoder wrappedEncoder;
    private boolean cachedConnected;
    private double cachedTemperatureCelsius;
    private int updateCycles = 0;

    public MotorControllerAdapter(MotorController raw, MotorControllerConfig config) {
        this.raw = raw;
        this.config = config != null ? config : raw.getConfig();
        Encoder encoder = raw.getEncoder();
        if (encoder != null) {
            EncoderConfig encoderConfig = this.config != null ? this.config.encoderConfig() : null;
            this.wrappedEncoder = EncoderAdapter.wrap(encoder, encoderConfig);
        }
        this.cachedConnected = true;
        this.cachedTemperatureCelsius = 0.0;
    }

    public static MotorController wrap(MotorController raw, MotorControllerConfig config) {
        if (raw == null) {
            return null;
        }
        if (raw instanceof MotorControllerAdapter) {
            return raw;
        }
        return new MotorControllerAdapter(raw, config);
    }

    @Override
    public int getId() {
        return raw.getId();
    }

    @Override
    public String getCanbus() {
        return raw.getCanbus();
    }

    @Override
    public MotorControllerType getType() {
        return raw.getType();
    }

    @Override
    public void setSpeed(double percent) {
        raw.setSpeed(isInverted() ? -percent : percent);
    }

    @Override
    public void setVoltage(double volts) {
        raw.setVoltage(isInverted() ? -volts : volts);
    }

    @Override
    public void setCurrentLimit(double amps) {
        if (config != null) {
            config.hardware().currentLimit(amps);
        }
        raw.setCurrentLimit(amps);
    }

    @Override
    public void setPosition(double rotations) {
        raw.setPosition(isInverted() ? -rotations : rotations);
    }

    @Override
    public void setVelocity(double rotationsPerSecond) {
        raw.setVelocity(isInverted() ? -rotationsPerSecond : rotationsPerSecond);
    }

    @Override
    public void setNeutralMode(MotorNeutralMode mode) {
        if (config != null) {
            config.hardware().neutralMode(mode);
        }
        raw.setNeutralMode(mode);
    }

    @Override
    public void setPid(PIDController pid) {
        if (config != null) {
            config.control().pid(pid);
        }
        raw.setPid(pid);
    }

    @Override
    public boolean isConnected() {
        return cachedConnected;
    }

    @Override
    public boolean isConnected(boolean poll) {
        if (poll) {
            cachedConnected = raw.isConnected();
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
            cachedTemperatureCelsius = raw.getTemperatureCelsius();
        }
        return cachedTemperatureCelsius;
    }

    @Override
    public Encoder getEncoder() {
        return wrappedEncoder;
    }

    @Override
    public boolean isInverted() {
        return config != null && config.inverted();
    }

    @Override
    public void setInverted(boolean inverted) {
        if (config != null) {
            config.hardware().inverted(inverted);
        }
    }

    @Override
    public double getCurrentLimit() {
        return config != null ? config.currentLimit() : raw.getCurrentLimit();
    }

    @Override
    public MotorNeutralMode getNeutralMode() {
        return config != null ? config.neutralMode() : raw.getNeutralMode();
    }

    @Override
    public MotorControllerConfig getConfig() {
        return config;
    }

    @Override
    public String getName() {
        return raw.getName();
    }

    @Override
    public void stopMotor() {
        raw.stopMotor();
    }

    @Override
    public void update() {
        raw.update();
        updateCycles++;
        if (updateCycles == 1 || updateCycles % CONNECTION_POLL_INTERVAL_CYCLES == 0) {
            cachedConnected = raw.isConnected();
        }
        if (updateCycles == 1 || updateCycles % TEMPERATURE_POLL_INTERVAL_CYCLES == 0) {
            cachedTemperatureCelsius = raw.getTemperatureCelsius();
        }
        if (updateCycles >= 1_000_000) {
            updateCycles = 0;
        }
    }
}
