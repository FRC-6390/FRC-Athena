package ca.frc6390.athena.rev.encoder;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import edu.wpi.first.wpilibj.RobotBase;

/**
 * REV encoder wrapper for the new vendordep system.
 */
public class RevEncoder implements Encoder {
    private final EncoderConfig config;
    private final RelativeEncoder relative;
    private final AbsoluteEncoder absolute;
    private double simPosition = 0.0;
    private double simVelocity = 0.0;

    public RevEncoder(RelativeEncoder relative, AbsoluteEncoder absolute, EncoderConfig config) {
        this.relative = relative;
        this.absolute = absolute;
        this.config = config;
        applyConfig(config);
    }

    private void applyConfig(EncoderConfig config) {
        if (config == null) {
            return;
        }
        // Raw encoder: configuration is applied by core adapter.
    }

    public static RevEncoder fromConfig(EncoderConfig config) {
        if (config == null || !(config.type() instanceof RevEncoderType)) {
            throw new IllegalArgumentException("REV encoder config required");
        }

        RevEncoderType type = (RevEncoderType) config.type();
        switch (type) {
            case SPARK_MAX -> {
                SparkMax motor = new SparkMax(config.id(), MotorType.kBrushless);
                return new RevEncoder(motor.getEncoder(), motor.getAbsoluteEncoder(), config);
            }
            case SPARK_FLEX -> {
                SparkFlex motor = new SparkFlex(config.id(), MotorType.kBrushless);
                return new RevEncoder(motor.getEncoder(), motor.getAbsoluteEncoder(), config);
            }
            default -> throw new IllegalArgumentException("Unsupported REV encoder type: " + type);
        }
    }

    @Override
    public double getPosition() {
        if (relative != null) {
            return relative.getPosition();
        }
        if (absolute != null) {
            return absolute.getPosition();
        }
        if (RobotBase.isSimulation()) {
            return simPosition;
        }
        return 0.0;
    }

    @Override
    public double getVelocity() {
        double rpm;
        if (relative != null) {
            rpm = relative.getVelocity();
        } else if (absolute != null) {
            rpm = absolute.getVelocity();
        } else if (RobotBase.isSimulation()) {
            return simVelocity;
        } else {
            return 0.0;
        }
        double rps = rpm / 60.0;
        return Double.isFinite(rps) ? rps : 0.0;
    }

    @Override
    public double getAbsolutePosition() {
        if (absolute != null) {
            return absolute.getPosition();
        }
        if (relative != null) {
            return relative.getPosition();
        }
        if (RobotBase.isSimulation()) {
            return simPosition;
        }
        return 0.0;
    }

    @Override
    public void setPosition(double position) {
        if (relative != null) {
            relative.setPosition(position);
        }
        if (RobotBase.isSimulation()) {
            simPosition = position;
        }
    }

    @Override
    public void setInverted(boolean inverted) {
        if (config != null) {
            config.hardware().inverted(inverted);
        }
    }

    @Override
    public void setConversion(double conversion) {
        if (config != null) {
            config.measurement().conversion(conversion);
        }
    }

    @Override
    public void setGearRatio(double gearRatio) {
        if (config != null) {
            config.measurement().gearRatio(gearRatio);
        }
    }

    @Override
    public void setConversionOffset(double conversionOffset) {
        if (config != null) {
            config.measurement().conversionOffset(conversionOffset);
        }
    }

    @Override
    public void setOffset(double offset) {
        if (config != null) {
            config.measurement().offset(offset);
        }
    }

    @Override
    public double getRawAbsoluteValue() {
        if (absolute != null) {
            return absolute.getPosition();
        }
        if (relative != null) {
            return relative.getPosition();
        }
        if (RobotBase.isSimulation()) {
            return simPosition;
        }
        return 0.0;
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
    public EncoderConfig getConfig() {
        return config;
    }
}
