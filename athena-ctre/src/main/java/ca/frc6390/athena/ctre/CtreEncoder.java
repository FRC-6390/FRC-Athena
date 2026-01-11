package ca.frc6390.athena.ctre;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import ca.frc6390.athena.hardware.Encoder;
import ca.frc6390.athena.hardware.EncoderConfig;

/**
 * CTRE encoder wrapper for the new vendordep system.
 */
public class CtreEncoder implements Encoder {
    private final EncoderConfig config;
    private final CANcoder cancoder;
    private final TalonFX talonFx;
    private double conversion = 1.0;
    private double offset = 0.0;
    private boolean inverted = false;

    public CtreEncoder(CANcoder cancoder, EncoderConfig config) {
        this.cancoder = cancoder;
        this.talonFx = null;
        this.config = config;
    }

    public CtreEncoder(TalonFX talonFx, EncoderConfig config) {
        this.talonFx = talonFx;
        this.cancoder = null;
        this.config = config;
    }

    public static CtreEncoder fromConfig(EncoderConfig config) {
        if (config == null || !(config.type instanceof CtreEncoderType)) {
            throw new IllegalArgumentException("CTRE encoder config required");
        }

        CtreEncoderType type = (CtreEncoderType) config.type;
        switch (type) {
            case CANCODER -> {
                CANcoder encoder = new CANcoder(config.id, config.canbus);
                encoder.getConfigurator().apply(new CANcoderConfiguration());
                return new CtreEncoder(encoder, config);
            }
            default -> throw new IllegalArgumentException("Unsupported CTRE encoder type: " + type);
        }
    }

    @Override
    public double getPosition() {
        double direction = inverted ? -1.0 : 1.0;
        if (cancoder != null) {
            return direction * (cancoder.getPosition().getValueAsDouble() + offset) * conversion;
        }
        if (talonFx != null) {
            return direction * (talonFx.getPosition().getValueAsDouble() + offset) * conversion;
        }
        return 0.0;
    }

    @Override
    public double getVelocity() {
        double direction = inverted ? -1.0 : 1.0;
        if (cancoder != null) {
            return direction * cancoder.getVelocity().getValueAsDouble() * conversion;
        }
        if (talonFx != null) {
            return direction * talonFx.getVelocity().getValueAsDouble() * conversion;
        }
        return 0.0;
    }

    @Override
    public void setPosition(double position) {
        double rawPosition = conversion != 0.0 ? (position / conversion) - offset : 0.0;
        if (cancoder != null) {
            cancoder.setPosition(rawPosition);
        } else if (talonFx != null) {
            talonFx.setPosition(rawPosition);
        }
    }

    @Override
    public void setInverted(boolean inverted) {
        this.inverted = inverted;
    }

    @Override
    public void setConversion(double conversion) {
        this.conversion = conversion;
    }

    @Override
    public void setOffset(double offset) {
        this.offset = offset;
    }

    @Override
    public EncoderConfig getConfig() {
        return config;
    }
}
