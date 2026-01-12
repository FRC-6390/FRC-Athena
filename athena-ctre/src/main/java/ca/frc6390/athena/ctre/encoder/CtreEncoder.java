package ca.frc6390.athena.ctre.encoder;

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
    private double conversion = 1.0;
    private double conversionOffset = 0.0;
    private double offset = 0.0;
    private double gearRatio = 1.0;
    private boolean inverted = false;
    private double simPosition = 0.0;
    private double simVelocity = 0.0;

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
        return fromConfig(config, null);
    }

    public static CtreEncoder fromConfig(EncoderConfig config, TalonFX talonFx) {
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
            case TALON_FX -> {
                if (talonFx == null) {
                    throw new IllegalArgumentException("TalonFX reference required for integrated encoder");
                }
                return new CtreEncoder(talonFx, config);
            }
            default -> throw new IllegalArgumentException("Unsupported CTRE encoder type: " + type);
        }
    }

    @Override
    public double getPosition() {
        double direction = inverted ? -1.0 : 1.0;
        if (RobotBase.isSimulation()) {
            return direction * (simPosition + offset) * conversion;
        }
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
        if (RobotBase.isSimulation()) {
            return direction * simVelocity * conversion;
        }
        if (cancoder != null) {
            return direction * cancoder.getVelocity().getValueAsDouble() * conversion;
        }
        if (talonFx != null) {
            return direction * talonFx.getVelocity().getValueAsDouble() * conversion;
        }
        return 0.0;
    }

    @Override
    public double getAbsolutePosition() {
        return getPosition(); // CANCoder absolute position already reflected via getPosition
    }

    @Override
    public double getAbsoluteRotations() {
        return getAbsolutePosition(); // same units as getPosition
    }

    @Override
    public double getRotations() {
        return getPosition(); // Talon/CANCoder expose rotations directly
    }

    @Override
    public double getRate() {
        return getVelocity();
    }

    @Override
    public double getConversion() {
        return conversion;
    }

    @Override
    public double getConversionOffset() {
        return conversionOffset;
    }

    @Override
    public double getOffset() {
        return offset;
    }

    @Override
    public double getGearRatio() {
        return gearRatio;
    }

    @Override
    public boolean isInverted() {
        return inverted;
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
        this.gearRatio = gearRatio;
    }

    @Override
    public void setConversionOffset(double conversionOffset) {
        this.conversionOffset = conversionOffset;
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
    public void setPosition(double position) {
        double rawPosition = conversion != 0.0 ? (position / conversion) - offset : 0.0;
        if (cancoder != null) {
            cancoder.setPosition(rawPosition);
        } else if (talonFx != null) {
            talonFx.setPosition(rawPosition);
        }
        if (RobotBase.isSimulation()) {
            simPosition = position;
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
