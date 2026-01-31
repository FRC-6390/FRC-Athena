package ca.frc6390.athena.hardware.encoder;

import ca.frc6390.athena.core.RobotSendableSystem;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

/**
 * Vendor-agnostic encoder interface for the new vendordep system.
 */
public interface Encoder extends RobotSendableSystem.RobotSendableDevice {
    double getPosition();

    double getVelocity();

    void setPosition(double position);

    void setInverted(boolean inverted);

    void setConversion(double conversion);

    void setOffset(double offset);

    // Expanded capability to match legacy usage
    default double getAbsolutePosition() {
        return getPosition();
    }

    default double getAbsoluteRotations() {
        return getRotations();
    }

    default double getRotations() {
        return getPosition();
    }

    default double getRate() {
        return getVelocity();
    }

    default double getConversion() {
        return 1.0;
    }

    default double getConversionOffset() {
        return 0.0;
    }

    default double getOffset() {
        return 0.0;
    }

    default double getDiscontinuityPoint() {
        return Double.NaN;
    }

    default double getDiscontinuityRange() {
        return Double.NaN;
    }

    default double getGearRatio() {
        return 1.0;
    }

    default boolean isInverted() {
        return false;
    }

    default boolean isConnected() {
        return true;
    }

    default void setGearRatio(double gearRatio) {}

    default void setConversionOffset(double conversionOffset) {}

    default void setDiscontinuityPoint(double discontinuityPoint) {}

    default void setDiscontinuityRange(double discontinuityRange) {}

    default void setDiscontinuity(double discontinuityPoint, double discontinuityRange) {
        setDiscontinuityPoint(discontinuityPoint);
        setDiscontinuityRange(discontinuityRange);
    }

    default void setRotations(double rotations) {
        setPosition(rotations);
    }

    // Simulation hooks
    default void setSimulatedPosition(double rotations) {}

    default void setSimulatedVelocity(double rotationsPerSecond) {}

    default void setSimulatedState(double rotations, double velocity) {}

    /**
     * Returns true when this encoder updates its readings from simulated state setters.
     */
    default boolean supportsSimulation() {
        return false;
    }

    // Additional helpers
    default edu.wpi.first.math.geometry.Rotation2d getRotation2d() {
        return edu.wpi.first.math.geometry.Rotation2d.fromRotations(getRotations());
    }

    default edu.wpi.first.math.geometry.Rotation2d getAbsoluteRotation2d() {
        return edu.wpi.first.math.geometry.Rotation2d.fromRotations(getAbsoluteRotations());
    }

    default Encoder update() {
        return this;
    }

    EncoderConfig getConfig();

    default double getRawAbsoluteValue() {
        return getAbsolutePosition();
    }

    // Default helpers to satisfy shuffleboard calls
    default String getName() {
        EncoderConfig cfg = getConfig();
        return cfg != null ? (cfg.canbus + "\\" + cfg.id + "\\" + (cfg.type != null ? cfg.type.getKey() : "encoder")) : "encoder";
    }

    @Override
    default ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, RobotSendableSystem.SendableLevel level) {
        layout.addDouble("Position", this::getPosition);
        layout.addDouble("Velocity", this::getVelocity);
        layout.addDouble("Absolute Position", this::getAbsolutePosition);
        layout.addBoolean("Connected", this::isConnected);
        if (level.equals(RobotSendableSystem.SendableLevel.DEBUG)) {
            layout.addDouble("Rotations", this::getRotations);
            layout.addDouble("Rate", this::getRate);
            layout.addDouble("Absolute Rotations", this::getAbsoluteRotations);
            layout.addDouble("Raw Absolute", this::getRawAbsoluteValue);
            layout.addDouble("Rotation (deg)", () -> getRotation2d().getDegrees());
            layout.addDouble("Absolute Rotation (deg)", () -> getAbsoluteRotation2d().getDegrees());
            layout.addBoolean("Supports Simulation", this::supportsSimulation);

            layout.add("Inverted", builder ->
                    builder.addBooleanProperty("Inverted", this::isInverted, this::setInverted));
            layout.add("Gear Ratio", builder ->
                    builder.addDoubleProperty("Gear Ratio", this::getGearRatio, this::setGearRatio));
            layout.add("Conversion", builder ->
                    builder.addDoubleProperty("Conversion", this::getConversion, this::setConversion));
            layout.add("Conversion Offset", builder ->
                    builder.addDoubleProperty("Conversion Offset", this::getConversionOffset, this::setConversionOffset));
            layout.add("Offset", builder ->
                    builder.addDoubleProperty("Offset", this::getOffset, this::setOffset));
            layout.add("Discontinuity Point", builder ->
                    builder.addDoubleProperty("Discontinuity Point", this::getDiscontinuityPoint, this::setDiscontinuityPoint));
            layout.add("Discontinuity Range", builder ->
                    builder.addDoubleProperty("Discontinuity Range", this::getDiscontinuityRange, this::setDiscontinuityRange));

            layout.addDouble("CAN ID", () -> {
                EncoderConfig cfg = getConfig();
                return cfg != null ? cfg.id : 0.0;
            });
            layout.addString("CAN Bus", () -> {
                EncoderConfig cfg = getConfig();
                return cfg != null && cfg.canbus != null ? cfg.canbus : "";
            });
            layout.addString("Type", () -> {
                EncoderConfig cfg = getConfig();
                return cfg != null && cfg.type != null ? cfg.type.getKey() : "unknown";
            });
        }
        return layout;
    }
}
