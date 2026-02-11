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

    /**
     * Returns the latest position reading. When {@code poll} is true, the encoder is refreshed
     * before the value is returned.
     */
    default double getPosition(boolean poll) {
        if (poll) {
            update();
        }
        return getPosition();
    }

    /**
     * Returns the latest velocity reading. When {@code poll} is true, the encoder is refreshed
     * before the value is returned.
     */
    default double getVelocity(boolean poll) {
        if (poll) {
            update();
        }
        return getVelocity();
    }

    void setPosition(double position);

    void setInverted(boolean inverted);

    void setConversion(double conversion);

    void setOffset(double offset);

    // Expanded capability to match legacy usage
    default double getAbsolutePosition() {
        return getPosition();
    }

    /**
     * Returns the latest absolute position reading. When {@code poll} is true, the encoder is
     * refreshed before the value is returned.
     */
    default double getAbsolutePosition(boolean poll) {
        if (poll) {
            update();
        }
        return getAbsolutePosition();
    }

    default double getAbsoluteRotations() {
        return getRotations();
    }

    default double getAbsoluteRotations(boolean poll) {
        if (poll) {
            update();
        }
        return getAbsoluteRotations();
    }

    default double getRotations() {
        return getPosition();
    }

    default double getRotations(boolean poll) {
        if (poll) {
            update();
        }
        return getRotations();
    }

    default double getRate() {
        return getVelocity();
    }

    default double getRate(boolean poll) {
        if (poll) {
            update();
        }
        return getRate();
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

    default boolean isConnected(boolean poll) {
        if (poll) {
            update();
        }
        return isConnected();
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

    default edu.wpi.first.math.geometry.Rotation2d getRotation2d(boolean poll) {
        return edu.wpi.first.math.geometry.Rotation2d.fromRotations(getRotations(poll));
    }

    default edu.wpi.first.math.geometry.Rotation2d getAbsoluteRotation2d() {
        return edu.wpi.first.math.geometry.Rotation2d.fromRotations(getAbsoluteRotations());
    }

    default edu.wpi.first.math.geometry.Rotation2d getAbsoluteRotation2d(boolean poll) {
        return edu.wpi.first.math.geometry.Rotation2d.fromRotations(getAbsoluteRotations(poll));
    }

    default Encoder update() {
        return this;
    }

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

    EncoderConfig getConfig();

    default double getRawAbsoluteValue() {
        return getAbsolutePosition();
    }

    default double getRawAbsoluteValue(boolean poll) {
        if (poll) {
            update();
        }
        return getRawAbsoluteValue();
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

        node.putDouble("position", getPosition());
        node.putDouble("velocity", getVelocity());
        node.putDouble("absolutePosition", getAbsolutePosition());
        node.putBoolean("connected", isConnected());

        if (nt.enabled(RobotNetworkTables.Flag.HW_ENCODER_TUNING_WIDGETS)) {
            node.putDouble("rotations", getRotations());
            node.putDouble("rate", getRate());
            node.putDouble("absoluteRotations", getAbsoluteRotations());
            node.putDouble("rawAbsolute", getRawAbsoluteValue());
            node.putDouble("rotationDeg", getRotation2d().getDegrees());
            node.putDouble("absoluteRotationDeg", getAbsoluteRotation2d().getDegrees());
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
