package ca.frc6390.athena.hardware.motor;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderAdapter;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import edu.wpi.first.math.controller.PIDController;

/**
 * Applies shared motor configuration (inversion, limits) on top of vendor-specific controllers.
 */
public class MotorControllerAdapter implements MotorController {
    private final MotorController raw;
    private final MotorControllerConfig config;
    private Encoder wrappedEncoder;
    private double shuffleboardPeriodSeconds = ca.frc6390.athena.core.RobotSendableSystem.getDefaultShuffleboardPeriodSeconds();
    private final int cachedId;
    private final String cachedCanbus;
    private final String cachedTypeKey;
    private boolean cachedConnected;
    private double cachedTemperatureCelsius;
    private MotorNeutralMode cachedNeutralMode;
    private double cachedCurrentLimit;
    private boolean cachedInverted;

    public MotorControllerAdapter(MotorController raw, MotorControllerConfig config) {
        this.raw = raw;
        this.config = config != null ? config : raw.getConfig();
        Encoder encoder = raw.getEncoder();
        if (encoder != null) {
            EncoderConfig encoderConfig = this.config != null ? this.config.encoderConfig : null;
            this.wrappedEncoder = EncoderAdapter.wrap(encoder, encoderConfig);
        }
        this.cachedId = raw.getId();
        this.cachedCanbus = raw.getCanbus();
        MotorControllerType type = raw.getType();
        this.cachedTypeKey = type != null ? type.getKey() : "unknown";
        this.cachedInverted = this.config != null && this.config.inverted;
        this.cachedCurrentLimit = this.config != null ? this.config.currentLimit : raw.getCurrentLimit();
        this.cachedNeutralMode = this.config != null ? this.config.neutralMode : raw.getNeutralMode();
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
            config.currentLimit = amps;
        }
        cachedCurrentLimit = amps;
        raw.setCurrentLimit(amps);
    }

    @Override
    public void setPosition(double rotations) {
        raw.setPosition(isInverted() ? -rotations : rotations);
    }

    @Override
    public void setNeutralMode(MotorNeutralMode mode) {
        if (config != null) {
            config.neutralMode = mode;
        }
        cachedNeutralMode = mode;
        raw.setNeutralMode(mode);
    }

    @Override
    public void setPid(PIDController pid) {
        if (config != null) {
            config.pid = pid;
        }
        raw.setPid(pid);
    }

    @Override
    public boolean isConnected() {
        return raw.isConnected();
    }

    @Override
    public double getTemperatureCelsius() {
        return raw.getTemperatureCelsius();
    }

    @Override
    public Encoder getEncoder() {
        return wrappedEncoder;
    }

    @Override
    public boolean isInverted() {
        return config != null && config.inverted;
    }

    @Override
    public void setInverted(boolean inverted) {
        if (config != null) {
            config.inverted = inverted;
        }
        cachedInverted = inverted;
    }

    @Override
    public double getCurrentLimit() {
        return config != null ? config.currentLimit : raw.getCurrentLimit();
    }

    @Override
    public MotorNeutralMode getNeutralMode() {
        return config != null ? config.neutralMode : raw.getNeutralMode();
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
        cachedConnected = raw.isConnected();
        cachedTemperatureCelsius = raw.getTemperatureCelsius();
    }

    @Override
    public double getShuffleboardPeriodSeconds() {
        return shuffleboardPeriodSeconds;
    }

    @Override
    public void setShuffleboardPeriodSeconds(double periodSeconds) {
        if (!Double.isFinite(periodSeconds)) {
            return;
        }
        shuffleboardPeriodSeconds = periodSeconds;
        raw.setShuffleboardPeriodSeconds(periodSeconds);
    }

    @Override
    public int getCachedId() {
        return cachedId;
    }

    @Override
    public String getCachedCanbus() {
        return cachedCanbus;
    }

    @Override
    public String getCachedTypeKey() {
        return cachedTypeKey;
    }

    @Override
    public boolean isCachedConnected() {
        return cachedConnected;
    }

    @Override
    public double getCachedTemperatureCelsius() {
        return cachedTemperatureCelsius;
    }

    @Override
    public MotorNeutralMode getCachedNeutralMode() {
        return cachedNeutralMode != null ? cachedNeutralMode : MotorNeutralMode.Coast;
    }

    @Override
    public double getCachedCurrentLimit() {
        return cachedCurrentLimit;
    }

    @Override
    public boolean isCachedInverted() {
        return cachedInverted;
    }
}
