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

    // Cached accessors for shuffleboard/logging (override in adapters to return cached values)
    default double getCachedPosition() { return getPosition(); }

    default double getCachedVelocity() { return getVelocity(); }

    default double getCachedAbsolutePosition() { return getAbsolutePosition(); }

    default double getCachedRotations() { return getRotations(); }

    default double getCachedRate() { return getRate(); }

    default double getCachedAbsoluteRotations() { return getAbsoluteRotations(); }

    default double getCachedRawAbsoluteValue() { return getRawAbsoluteValue(); }

    default double getCachedRotationDegrees() { return getRotation2d().getDegrees(); }

    default double getCachedAbsoluteRotationDegrees() { return getAbsoluteRotation2d().getDegrees(); }

    default boolean isCachedConnected() { return isConnected(); }

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
        java.util.function.DoubleSupplier period = this::getShuffleboardPeriodSeconds;
        layout.addDouble("Position", RobotSendableSystem.rateLimit(this::getCachedPosition, period));
        layout.addDouble("Velocity", RobotSendableSystem.rateLimit(this::getCachedVelocity, period));
        layout.addDouble("Absolute Position", RobotSendableSystem.rateLimit(this::getCachedAbsolutePosition, period));
        layout.addBoolean("Connected", RobotSendableSystem.rateLimit(this::isCachedConnected, period));
        if (level.equals(RobotSendableSystem.SendableLevel.DEBUG)) {
            layout.addDouble("Rotations", RobotSendableSystem.rateLimit(this::getCachedRotations, period));
            layout.addDouble("Rate", RobotSendableSystem.rateLimit(this::getCachedRate, period));
            layout.addDouble("Absolute Rotations", RobotSendableSystem.rateLimit(this::getCachedAbsoluteRotations, period));
            layout.addDouble("Raw Absolute", RobotSendableSystem.rateLimit(this::getCachedRawAbsoluteValue, period));
            layout.addDouble("Rotation (deg)", RobotSendableSystem.rateLimit(this::getCachedRotationDegrees, period));
            layout.addDouble("Absolute Rotation (deg)", RobotSendableSystem.rateLimit(this::getCachedAbsoluteRotationDegrees, period));
            layout.addBoolean("Supports Simulation", RobotSendableSystem.rateLimit(this::supportsSimulation, period));

            layout.add("Inverted", builder ->
                    builder.addBooleanProperty(
                            "Inverted",
                            RobotSendableSystem.rateLimit(this::isInverted, period),
                            this::setInverted));
            layout.add("Gear Ratio", builder ->
                    builder.addDoubleProperty(
                            "Gear Ratio",
                            RobotSendableSystem.rateLimit(this::getGearRatio, period),
                            this::setGearRatio));
            layout.add("Conversion", builder ->
                    builder.addDoubleProperty(
                            "Conversion",
                            RobotSendableSystem.rateLimit(this::getConversion, period),
                            this::setConversion));
            layout.add("Conversion Offset", builder ->
                    builder.addDoubleProperty(
                            "Conversion Offset",
                            RobotSendableSystem.rateLimit(this::getConversionOffset, period),
                            this::setConversionOffset));
            layout.add("Offset", builder ->
                    builder.addDoubleProperty(
                            "Offset",
                            RobotSendableSystem.rateLimit(this::getOffset, period),
                            this::setOffset));
            layout.add("Discontinuity Point", builder ->
                    builder.addDoubleProperty(
                            "Discontinuity Point",
                            RobotSendableSystem.rateLimit(this::getDiscontinuityPoint, period),
                            this::setDiscontinuityPoint));
            layout.add("Discontinuity Range", builder ->
                    builder.addDoubleProperty(
                            "Discontinuity Range",
                            RobotSendableSystem.rateLimit(this::getDiscontinuityRange, period),
                            this::setDiscontinuityRange));

            layout.addDouble("CAN ID", RobotSendableSystem.rateLimit(() -> {
                EncoderConfig cfg = getConfig();
                return cfg != null ? cfg.id : 0.0;
            }, period));
            layout.addString("CAN Bus", RobotSendableSystem.rateLimit(() -> {
                EncoderConfig cfg = getConfig();
                return cfg != null && cfg.canbus != null ? cfg.canbus : "";
            }, period));
            layout.addString("Type", RobotSendableSystem.rateLimit(() -> {
                EncoderConfig cfg = getConfig();
                return cfg != null && cfg.type != null ? cfg.type.getKey() : "unknown";
            }, period));
        }
        return layout;
    }
}
