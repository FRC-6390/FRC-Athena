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
        layout.addDouble("CAN ID", () -> getId());
        layout.addString("CAN Bus", this::getCanbus);
        layout.addString("Type", () -> {
            MotorControllerType type = getType();
            return type != null ? type.getKey() : "unknown";
        });
        layout.addBoolean("Connected", this::isConnected);
        layout.addDouble("Temperature (C)", this::getTemperatureCelsius);
        layout.addString("Neutral Mode", () -> getNeutralMode().name());
        if (level.equals(RobotSendableSystem.SendableLevel.DEBUG)) {
            layout.add("Current Limit (A)", builder ->
                    builder.addDoubleProperty("Current Limit (A)", this::getCurrentLimit, this::setCurrentLimit));
            layout.add("Inverted", builder ->
                    builder.addBooleanProperty("Inverted", this::isInverted, this::setInverted));
            layout.add("Brake Mode", builder ->
                    builder.addBooleanProperty(
                            "Brake Mode",
                            () -> getNeutralMode() == MotorNeutralMode.Brake,
                            value -> setNeutralMode(value ? MotorNeutralMode.Brake : MotorNeutralMode.Coast)));
            Encoder encoder = getEncoder();
            if (encoder != null) {
                encoder.shuffleboard(layout.getLayout("Encoder", BuiltInLayouts.kList), level);
            }
        }
        return layout;
    }
}
