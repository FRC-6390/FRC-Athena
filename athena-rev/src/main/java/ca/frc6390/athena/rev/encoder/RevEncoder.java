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
    private double conversionOffset = 0.0;
    private double gearRatio = 1.0;
    private boolean inverted = false;
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
        this.conversion = config.conversion;
        this.conversionOffset = config.conversionOffset;
        this.offset = config.offset;
        this.gearRatio = config.gearRatio;
        this.inverted = config.inverted;
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
            return direction * ((simPosition * gearRatio + offset) * conversion - conversionOffset);
        }
        if (absolute != null) {
            double rawPosition = absolute.getPosition();
            return direction * ((rawPosition * gearRatio + offset) * conversion - conversionOffset);
        }
        double rawPosition = relative.getPosition();
        return direction * ((rawPosition * gearRatio + offset) * conversion - conversionOffset);
    }

    @Override
    public double getVelocity() {
        double direction = inverted ? -1.0 : 1.0;
        if (RobotBase.isSimulation()) {
            return direction * simVelocity * gearRatio * conversion;
        }
        if (absolute != null) {
            return direction * absolute.getVelocity() * gearRatio * conversion;
        }
        return direction * relative.getVelocity() * gearRatio * conversion;
    }

    @Override
    public void setPosition(double position) {
        double safeConversion = conversion != 0.0 ? conversion : 1.0;
        double safeGearRatio = gearRatio != 0.0 ? gearRatio : 1.0;
        double raw = ((position + conversionOffset) / safeConversion - offset) / safeGearRatio;
        relative.setPosition(raw);
        if (RobotBase.isSimulation()) {
            simPosition = raw;
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
    public void setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
    }

    @Override
    public void setConversionOffset(double conversionOffset) {
        this.conversionOffset = conversionOffset;
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
    public boolean supportsSimulation() {
        return true;
    }

    @Override
    public EncoderConfig getConfig() {
        return config;
    }
}
