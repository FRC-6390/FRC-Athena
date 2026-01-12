package ca.frc6390.athena.hardware.motor;

import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.hardware.encoder.Encoder;
import edu.wpi.first.math.controller.PIDController;
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

    // Compatibility helpers
    default MotorControllerType getMotorControllerType() { return getType(); }

    @Override
    default ShuffleboardLayout shuffleboard(ShuffleboardLayout layout, RobotSendableSystem.SendableLevel level) {
        return layout;
    }
}
