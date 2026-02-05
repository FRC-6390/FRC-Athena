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

    default MotorControllerConfig getConfig() { return null; }

    // Compatibility helpers
    default MotorControllerType getMotorControllerType() { return getType(); }

    @Override
    default ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, RobotSendableSystem.SendableLevel level) {
        java.util.function.DoubleSupplier period = this::getShuffleboardPeriodSeconds;
        layout.addDouble("CAN ID", RobotSendableSystem.rateLimit(this::getId, period));
        layout.addString("CAN Bus", RobotSendableSystem.rateLimit(this::getCanbus, period));
        layout.addString("Type", RobotSendableSystem.rateLimit(() -> {
            MotorControllerType type = getType();
            return type != null ? type.getKey() : "unknown";
        }, period));
        layout.addBoolean("Connected", RobotSendableSystem.rateLimit(this::isConnected, period));
        layout.addDouble("Temperature (C)", RobotSendableSystem.rateLimit(this::getTemperatureCelsius, period));
        layout.addString("Neutral Mode", RobotSendableSystem.rateLimit(() -> getNeutralMode().name(), period));
        if (level.equals(RobotSendableSystem.SendableLevel.DEBUG)) {
            layout.add("Current Limit (A)", builder ->
                    builder.addDoubleProperty(
                            "Current Limit (A)",
                            RobotSendableSystem.rateLimit(this::getCurrentLimit, period),
                            this::setCurrentLimit));
            layout.add("Inverted", builder ->
                    builder.addBooleanProperty(
                            "Inverted",
                            RobotSendableSystem.rateLimit(this::isInverted, period),
                            this::setInverted));
            layout.add("Brake Mode", builder ->
                    builder.addBooleanProperty(
                            "Brake Mode",
                            RobotSendableSystem.rateLimit(
                                    () -> getNeutralMode() == MotorNeutralMode.Brake,
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
