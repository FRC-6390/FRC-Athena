package ca.frc6390.athena.hardware.encoder;

import java.util.function.DoubleSupplier;

/**
 * Simple encoder backed by caller-supplied functions in mechanism-space units.
 */
public class SupplierEncoder implements Encoder {
    private final EncoderConfig config;
    private final DoubleSupplier positionSupplier;
    private final DoubleSupplier velocitySupplier;
    private final DoubleSupplier absoluteSupplier;
    private double simulatedPosition;
    private double simulatedVelocity;
    private boolean useSimulation;

    public SupplierEncoder(
            EncoderConfig config,
            DoubleSupplier positionSupplier,
            DoubleSupplier velocitySupplier,
            DoubleSupplier absoluteSupplier) {
        this.config = config != null ? config : EncoderConfig.create();
        this.positionSupplier = positionSupplier;
        this.velocitySupplier = velocitySupplier;
        this.absoluteSupplier = absoluteSupplier;
    }

    @Override
    public double getPosition() {
        if (useSimulation) {
            return simulatedPosition;
        }
        return positionSupplier != null ? positionSupplier.getAsDouble() : 0.0;
    }

    @Override
    public double getVelocity() {
        if (useSimulation) {
            return simulatedVelocity;
        }
        return velocitySupplier != null ? velocitySupplier.getAsDouble() : 0.0;
    }

    @Override
    public double getAbsolutePosition() {
        if (useSimulation) {
            return simulatedPosition;
        }
        if (absoluteSupplier != null) {
            return absoluteSupplier.getAsDouble();
        }
        return getPosition();
    }

    @Override
    public void setPosition(double position) {
        simulatedPosition = position;
        useSimulation = true;
    }

    @Override
    public void setInverted(boolean inverted) {
    }

    @Override
    public void setConversion(double conversion) {
    }

    @Override
    public void setOffset(double offset) {
    }

    @Override
    public void setSimulatedPosition(double rotations) {
        simulatedPosition = rotations;
        useSimulation = true;
    }

    @Override
    public void setSimulatedVelocity(double rotationsPerSecond) {
        simulatedVelocity = rotationsPerSecond;
        useSimulation = true;
    }

    @Override
    public void setSimulatedState(double rotations, double velocity) {
        simulatedPosition = rotations;
        simulatedVelocity = velocity;
        useSimulation = true;
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
