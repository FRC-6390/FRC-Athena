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
    private double conversion = 1.0;
    private double offset = 0.0;
    private boolean inverted = false;
    private double simPosition = 0.0;
    private double simVelocity = 0.0;

    public RevEncoder(RelativeEncoder relative, AbsoluteEncoder absolute, EncoderConfig config) {
        this.relative = relative;
        this.absolute = absolute;
        this.config = config;
    }

    public static RevEncoder fromConfig(EncoderConfig config) {
        if (config == null || !(config.type instanceof RevEncoderType)) {
            throw new IllegalArgumentException("REV encoder config required");
        }

        RevEncoderType type = (RevEncoderType) config.type;
        switch (type) {
            case SPARK_MAX -> {
                SparkMax motor = new SparkMax(config.id, MotorType.kBrushless);
                return new RevEncoder(motor.getEncoder(), motor.getAbsoluteEncoder(), config);
            }
            case SPARK_FLEX -> {
                SparkFlex motor = new SparkFlex(config.id, MotorType.kBrushless);
                return new RevEncoder(motor.getEncoder(), motor.getAbsoluteEncoder(), config);
            }
            default -> throw new IllegalArgumentException("Unsupported REV encoder type: " + type);
        }
    }

    @Override
    public double getPosition() {
        double direction = inverted ? -1.0 : 1.0;
        if (RobotBase.isSimulation()) {
            return direction * (simPosition + offset) * conversion;
        }
        if (absolute != null) {
            return direction * (absolute.getPosition() + offset) * conversion;
        }
        return direction * (relative.getPosition() + offset) * conversion;
    }

    @Override
    public double getVelocity() {
        double direction = inverted ? -1.0 : 1.0;
        if (RobotBase.isSimulation()) {
            return direction * simVelocity * conversion;
        }
        if (absolute != null) {
            return direction * absolute.getVelocity() * conversion;
        }
        return direction * relative.getVelocity() * conversion;
    }

    @Override
    public void setPosition(double position) {
        double raw = conversion != 0.0 ? (position / conversion) - offset : 0.0;
        relative.setPosition(raw);
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
    public EncoderConfig getConfig() {
        return config;
    }
}
