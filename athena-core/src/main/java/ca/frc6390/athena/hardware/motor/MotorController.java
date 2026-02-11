package ca.frc6390.athena.hardware.motor;

import ca.frc6390.athena.core.RobotSendableSystem;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.core.RobotNetworkTables;
import edu.wpi.first.math.controller.PIDController;

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

    default boolean isConnected(boolean poll) {
        if (poll) {
            update();
        }
        return isConnected();
    }

    default double getTemperatureCelsius(boolean poll) {
        if (poll) {
            update();
        }
        return getTemperatureCelsius();
    }

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
    default RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return node;
        }
        RobotNetworkTables nt = node.robot();
        if (!nt.isPublishingEnabled()) {
            return node;
        }

        MotorControllerType type = getType();
        node.putDouble("canId", getId());
        node.putString("canbus", getCanbus());
        node.putString("type", type != null ? type.getKey() : "unknown");
        node.putBoolean("connected", isConnected());
        node.putDouble("tempC", getTemperatureCelsius());
        node.putString("neutralMode", getNeutralMode().name());

        if (nt.enabled(RobotNetworkTables.Flag.HW_MOTOR_TUNING_WIDGETS)) {
            node.putDouble("currentLimitA", getCurrentLimit());
            node.putBoolean("inverted", isInverted());
            node.putBoolean("brakeMode", getNeutralMode() == MotorNeutralMode.Brake);

            Encoder encoder = getEncoder();
            if (encoder != null) {
                encoder.networkTables(node.child("Encoder"));
            }
        }

        return node;
    }
}
