package ca.frc6390.athena.hardware.encoder;

import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.core.RobotNetworkTables;
import edu.wpi.first.math.MathUtil;

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

    // Cached accessors for logging/dashboard (override in adapters to return cached values)
    default double getCachedPosition() { return getPosition(); }

    default double getCachedVelocity() { return getVelocity(); }

    default double getCachedAbsolutePosition() { return getAbsolutePosition(); }

    /**
     * Returns the current position wrapped into the provided range.
     *
     * This is useful for "angle-like" mechanisms (turrets, wrists, etc.) where you want a stable
     * 0..360 (or -180..180) view for logging/UI, while still keeping {@link #getPosition()} as a
     * continuous multi-turn signal for control.
     *
     * Range is interpreted in the same units as {@link #getPosition()} (after gear ratio and
     * conversion have been applied by the encoder implementation).
     */
    default double getPositionModulus(double min, double max) {
        return MathUtil.inputModulus(getPosition(), min, max);
    }

    /**
     * Cached variant of {@link #getPositionModulus(double, double)} for logging/UI.
     */
    default double getCachedPositionModulus(double min, double max) {
        return MathUtil.inputModulus(getCachedPosition(), min, max);
    }

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
    default RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return node;
        }
        RobotNetworkTables nt = node.robot();
        if (!nt.isPublishingEnabled()) {
            return node;
        }

        node.putDouble("position", getCachedPosition());
        node.putDouble("velocity", getCachedVelocity());
        node.putDouble("absolutePosition", getCachedAbsolutePosition());
        node.putBoolean("connected", isCachedConnected());

        if (nt.enabled(RobotNetworkTables.Flag.HW_ENCODER_TUNING_WIDGETS)) {
            node.putDouble("rotations", getCachedRotations());
            node.putDouble("rate", getCachedRate());
            node.putDouble("absoluteRotations", getCachedAbsoluteRotations());
            node.putDouble("rawAbsolute", getCachedRawAbsoluteValue());
            node.putDouble("rotationDeg", getCachedRotationDegrees());
            node.putDouble("absoluteRotationDeg", getCachedAbsoluteRotationDegrees());
            node.putBoolean("supportsSimulation", supportsSimulation());

            EncoderConfig cfg = getConfig();
            if (cfg != null) {
                node.putDouble("canId", cfg.id);
                node.putString("canbus", cfg.canbus != null ? cfg.canbus : "");
                node.putString("type", cfg.type != null ? cfg.type.getKey() : "unknown");
            }
        }

        return node;
    }
}
