package ca.frc6390.athena.ctre.encoder;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * CTRE encoder wrapper for the new vendordep system.
 */
public class CtreEncoder implements Encoder {
    private final EncoderConfig config;
    private final CANcoder cancoder;
    private final TalonFX talonFx;
    private final StatusSignal<?> positionSignal;
    private final StatusSignal<?> velocitySignal;
    private final StatusSignal<?> absolutePositionSignal;
    private final BaseStatusSignal[] refreshSignals;
    private double simPosition = 0.0;
    private double simVelocity = 0.0;
    private double cachedPosition = 0.0;
    private double cachedVelocity = 0.0;
    private double cachedAbsolutePosition = 0.0;
    private boolean cachedConnected = false;

    public CtreEncoder(CANcoder cancoder, EncoderConfig config) {
        this.cancoder = cancoder;
        this.talonFx = null;
        this.config = config;
        this.positionSignal = cancoder.getPosition(false).clone();
        this.velocitySignal = cancoder.getVelocity(false).clone();
        this.absolutePositionSignal = cancoder.getAbsolutePosition(false).clone();
        this.refreshSignals = new BaseStatusSignal[] { positionSignal, velocitySignal, absolutePositionSignal };
        applyConfig(config);
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, positionSignal, velocitySignal, absolutePositionSignal);
        cancoder.optimizeBusUtilization(4.0);
        update();
    }

    public CtreEncoder(TalonFX talonFx, EncoderConfig config) {
        this.talonFx = talonFx;
        this.cancoder = null;
        this.config = config;
        this.positionSignal = talonFx.getPosition(false).clone();
        this.velocitySignal = talonFx.getVelocity(false).clone();
        this.absolutePositionSignal = positionSignal;
        this.refreshSignals = new BaseStatusSignal[] { positionSignal, velocitySignal };
        applyConfig(config);
        BaseStatusSignal.setUpdateFrequencyForAll(100.0, positionSignal, velocitySignal);
        talonFx.optimizeBusUtilization(4.0);
        update();
    }

    private void applyConfig(EncoderConfig config) {
        if (config == null || cancoder == null) {
            return;
        }
        boolean hasDiscontinuityConfig =
                Double.isFinite(config.discontinuityPoint) || Double.isFinite(config.discontinuityRange);

        if (!hasDiscontinuityConfig) {
            CANcoderConfiguration current = new CANcoderConfiguration();
            cancoder.getConfigurator().refresh(current);
            config.discontinuityRange = 1.0;
            config.discontinuityPoint = current.MagnetSensor.AbsoluteSensorDiscontinuityPoint;
            return;
        }

        double range = Double.isFinite(config.discontinuityRange)
                ? config.discontinuityRange
                : (Double.isFinite(config.discontinuityPoint) ? 1.0 : Double.NaN);
        if (Double.isFinite(range) && range > 0.0) {
            if (Math.abs(range - 1.0) > 1e-4) {
                DriverStation.reportWarning(
                        "CANCoder discontinuity range must be 1.0 rotation for hardware config; "
                                + "requested " + range + ". Using software wrap only.",
                        false);
                return;
            }
            if (!Double.isFinite(config.discontinuityPoint)) {
                DriverStation.reportWarning(
                        "CANCoder discontinuity point missing; set encoder discontinuity point to apply hardware wrap.",
                        false);
                return;
            }
            CANcoderConfiguration cfg = new CANcoderConfiguration();
            cfg.MagnetSensor.AbsoluteSensorDiscontinuityPoint =
                    MathUtil.clamp(config.discontinuityPoint, 0.0, 1.0);
            cancoder.getConfigurator().apply(cfg);
        }
    }

    public static CtreEncoder fromConfig(EncoderConfig config) {
        return fromConfig(config, null);
    }

    public static CtreEncoder fromConfig(EncoderConfig config, TalonFX talonFx) {
        if (config == null || !(config.type instanceof CtreEncoderType)) {
            throw new IllegalArgumentException("CTRE encoder config required");
        }

        CtreEncoderType type = (CtreEncoderType) config.type;
        switch (type) {
            case CANCODER -> {
                CANcoder encoder = new CANcoder(config.id, resolveCanBus(config.canbus));
                return new CtreEncoder(encoder, config);
            }
            case TALON_FX -> {
                if (talonFx == null) {
                    throw new IllegalArgumentException("TalonFX reference required for integrated encoder");
                }
                return new CtreEncoder(talonFx, config);
            }
            default -> throw new IllegalArgumentException("Unsupported CTRE encoder type: " + type);
        }
    }

    private static CANBus resolveCanBus(String canbus) {
        if (canbus == null || canbus.isBlank()) {
            return new CANBus();
        }
        return new CANBus(canbus);
    }

    @Override
    public double getPosition() {
        if (RobotBase.isSimulation()) {
            return simPosition;
        }
        return cachedPosition;
    }

    @Override
    public double getVelocity() {
        if (RobotBase.isSimulation()) {
            return simVelocity;
        }
        return cachedVelocity;
    }

    @Override
    public double getAbsolutePosition() {
        if (RobotBase.isSimulation()) {
            return simPosition;
        }
        return cachedAbsolutePosition;
    }

    @Override
    public double getAbsoluteRotations() {
        return getAbsolutePosition();
    }

    @Override
    public double getRotations() {
        return getPosition();
    }

    @Override
    public double getRate() {
        return getVelocity();
    }

    @Override
    public boolean isConnected() {
        if (RobotBase.isSimulation()) {
            return true;
        }
        return cachedConnected;
    }

    @Override
    public Encoder update() {
        if (RobotBase.isSimulation()) {
            cachedPosition = simPosition;
            cachedVelocity = simVelocity;
            cachedAbsolutePosition = simPosition;
            cachedConnected = true;
            return this;
        }

        BaseStatusSignal.refreshAll(refreshSignals);
        cachedPosition = positionSignal.getValueAsDouble();
        cachedVelocity = velocitySignal.getValueAsDouble();
        cachedAbsolutePosition = absolutePositionSignal.getValueAsDouble();
        if (cancoder != null) {
            cachedConnected = cancoder.isConnected();
            return this;
        }
        if (talonFx != null) {
            cachedConnected = talonFx.isConnected();
            return this;
        }
        cachedConnected = false;
        return this;
    }

    @Override
    public void setGearRatio(double gearRatio) {
        if (config != null) {
            config.gearRatio = gearRatio;
        }
    }

    @Override
    public void setConversionOffset(double conversionOffset) {
        if (config != null) {
            config.conversionOffset = conversionOffset;
        }
    }

    @Override
    public void setRotations(double rotations) {
        setPosition(rotations);
    }

    @Override
    public void setSimulatedPosition(double rotations) {
        simPosition = rotations;
        cachedPosition = rotations;
        cachedAbsolutePosition = rotations;
    }

    @Override
    public void setSimulatedVelocity(double rotationsPerSecond) {
        simVelocity = rotationsPerSecond;
        cachedVelocity = rotationsPerSecond;
    }

    @Override
    public void setSimulatedState(double rotations, double velocity) {
        simPosition = rotations;
        simVelocity = velocity;
        cachedPosition = rotations;
        cachedAbsolutePosition = rotations;
        cachedVelocity = velocity;
    }

    @Override
    public boolean supportsSimulation() {
        return true;
    }

    @Override
    public void setPosition(double position) {
        if (cancoder != null) {
            cancoder.setPosition(position);
        } else if (talonFx != null) {
            talonFx.setPosition(position);
        }
        if (RobotBase.isSimulation()) {
            simPosition = position;
            cachedPosition = position;
            cachedAbsolutePosition = position;
        }
    }

    @Override
    public void setInverted(boolean inverted) {
        if (config != null) {
            config.inverted = inverted;
        }
    }

    @Override
    public void setConversion(double conversion) {
        if (config != null) {
            config.conversion = conversion;
        }
    }

    @Override
    public void setOffset(double offset) {
        if (config != null) {
            config.offset = offset;
        }
    }

    @Override
    public EncoderConfig getConfig() {
        return config;
    }
}
