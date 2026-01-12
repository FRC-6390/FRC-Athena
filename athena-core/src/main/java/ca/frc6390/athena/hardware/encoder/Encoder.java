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

    default void setRotations(double rotations) {
        setPosition(rotations);
    }

    // Simulation hooks
    default void setSimulatedPosition(double rotations) {}

    default void setSimulatedVelocity(double rotationsPerSecond) {}

    default void setSimulatedState(double rotations, double velocity) {}

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
        return layout;
    }
}
