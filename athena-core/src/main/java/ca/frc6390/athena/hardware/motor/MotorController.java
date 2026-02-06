package ca.frc6390.athena.hardware.motor;

import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.hardware.encoder.Encoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

/**
 * Vendor-agnostic motor controller interface for the new vendordep system.
 */
public interface MotorController extends RobotSendableSystem.RobotSendableDevice {
    // Identity
    int getId();

    String getCanbus();

    MotorControllerType getType();

    // Outputs
    void setSpeed(double percent);

    void setVoltage(double volts);

    void setCurrentLimit(double amps);

    void setPosition(double rotations);

    void setVelocity(double rotationsPerSecond);

    void setNeutralMode(MotorNeutralMode mode);

    void setPid(PIDController pid);

    boolean isConnected();

    double getTemperatureCelsius();

    Encoder getEncoder();

    // Expanded API to match legacy usage
    default boolean isInverted() { return false; }

    default void setInverted(boolean inverted) {}

    default double getCurrentLimit() { return 0.0; }

    default MotorNeutralMode getNeutralMode() { return MotorNeutralMode.Coast; }

    default String getName() { return getCanbus() + "\\" + getId() + "\\" + getType().getKey(); }

    default void stopMotor() { setSpeed(0); }

    default void update() {}

    // Cached accessors for shuffleboard/logging (override in adapters to return cached values)
    default int getCachedId() { return getId(); }

    default String getCachedCanbus() { return getCanbus(); }

    default String getCachedTypeKey() {
        MotorControllerType type = getType();
        return type != null ? type.getKey() : "unknown";
    }

    default boolean isCachedConnected() { return isConnected(); }

    default double getCachedTemperatureCelsius() { return getTemperatureCelsius(); }

    default MotorNeutralMode getCachedNeutralMode() { return getNeutralMode(); }

    default double getCachedCurrentLimit() { return getCurrentLimit(); }

    default boolean isCachedInverted() { return isInverted(); }

    default MotorControllerConfig getConfig() { return null; }

    // Compatibility helpers
    default MotorControllerType getMotorControllerType() { return getType(); }

    @Override
    default ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, RobotSendableSystem.SendableLevel level) {
        java.util.function.DoubleSupplier period = this::getShuffleboardPeriodSeconds;
        layout.addDouble("CAN ID", RobotSendableSystem.rateLimit(this::getCachedId, period));
        layout.addString("CAN Bus", RobotSendableSystem.rateLimit(this::getCachedCanbus, period));
        layout.addString("Type", RobotSendableSystem.rateLimit(this::getCachedTypeKey, period));
        layout.addBoolean("Connected", RobotSendableSystem.rateLimit(this::isCachedConnected, period));
        layout.addDouble("Temperature (C)", RobotSendableSystem.rateLimit(this::getCachedTemperatureCelsius, period));
        layout.addString("Neutral Mode", RobotSendableSystem.rateLimit(() -> getCachedNeutralMode().name(), period));
        if (level.equals(RobotSendableSystem.SendableLevel.DEBUG)) {
            layout.add("Current Limit (A)", builder ->
                    builder.addDoubleProperty(
                            "Current Limit (A)",
                            RobotSendableSystem.rateLimit(this::getCachedCurrentLimit, period),
                            this::setCurrentLimit));
            layout.add("Inverted", builder ->
                    builder.addBooleanProperty(
                            "Inverted",
                            RobotSendableSystem.rateLimit(this::isCachedInverted, period),
                            this::setInverted));
            layout.add("Brake Mode", builder ->
                    builder.addBooleanProperty(
                            "Brake Mode",
                            RobotSendableSystem.rateLimit(
                                    () -> getCachedNeutralMode() == MotorNeutralMode.Brake,
                                    period),
                            value -> setNeutralMode(value ? MotorNeutralMode.Brake : MotorNeutralMode.Coast)));
            Encoder encoder = getEncoder();
            if (encoder != null) {
                encoder.shuffleboard(layout.getLayout("Encoder", BuiltInLayouts.kList), level);
            }
        }
        return layout;
    }
}
