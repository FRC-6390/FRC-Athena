package ca.frc6390.athena.ctre.encoder;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * CTRE encoder wrapper for the new vendordep system.
 */
public class CtreEncoder implements Encoder {
    private final EncoderConfig config;
    private final CANcoder cancoder;
    private final TalonFX talonFx;
    private double simPosition = 0.0;
    private double simVelocity = 0.0;

    public CtreEncoder(CANcoder cancoder, EncoderConfig config) {
        this.cancoder = cancoder;
        this.talonFx = null;
        this.config = config;
        applyConfig(config);
    }

    public CtreEncoder(TalonFX talonFx, EncoderConfig config) {
        this.talonFx = talonFx;
        this.cancoder = null;
        this.config = config;
        applyConfig(config);
    }

    private void applyConfig(EncoderConfig config) {
        if (config == null) {
            return;
        }
        // Raw encoder: configuration is applied by core adapter.
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
                encoder.getConfigurator().apply(new CANcoderConfiguration());
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
        if (cancoder != null) {
            return cancoder.getPosition().getValueAsDouble();
        }
        if (talonFx != null) {
            return talonFx.getPosition().getValueAsDouble();
        }
        return 0.0;
    }

    @Override
    public double getVelocity() {
        if (RobotBase.isSimulation()) {
            return simVelocity;
        }
        if (cancoder != null) {
            return cancoder.getVelocity().getValueAsDouble();
        }
        if (talonFx != null) {
            return talonFx.getVelocity().getValueAsDouble();
        }
        return 0.0;
    }

    @Override
    public double getAbsolutePosition() {
        if (RobotBase.isSimulation()) {
            return simPosition;
        }
        if (cancoder != null) {
            return cancoder.getAbsolutePosition().getValueAsDouble();
        }
        if (talonFx != null) {
            return talonFx.getPosition().getValueAsDouble();
        }
        return 0.0;
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
        if (cancoder != null) {
            return cancoder.isConnected();
        }
        if (talonFx != null) {
            return talonFx.isConnected();
        }
        return false;
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
    }

    @Override
    public void setSimulatedVelocity(double rotationsPerSecond) {
        simVelocity = rotationsPerSecond;
    }

    @Override
    public void setSimulatedState(double rotations, double velocity) {
        simPosition = rotations;
        simVelocity = velocity;
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
