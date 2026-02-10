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

    Encoder getEncoder();

    // Expanded API to match legacy usage
    default boolean isInverted() { return false; }

    default void setInverted(boolean inverted) {}

    default double getCurrentLimit() { return 0.0; }

    default MotorNeutralMode getNeutralMode() { return MotorNeutralMode.Coast; }

    default String getName() { return getCanbus() + "\\" + getId() + "\\" + getType().getKey(); }

    default void stopMotor() { setSpeed(0); }

    default void update() {}

    // Cached accessors for logging/dashboard (override in adapters to return cached values)
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
    default RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return node;
        }
        RobotNetworkTables nt = node.robot();
        if (!nt.isPublishingEnabled()) {
            return node;
        }

        node.putDouble("canId", getCachedId());
        node.putString("canbus", getCachedCanbus());
        node.putString("type", getCachedTypeKey());
        node.putBoolean("connected", isCachedConnected());
        node.putDouble("tempC", getCachedTemperatureCelsius());
        node.putString("neutralMode", getCachedNeutralMode().name());

        if (nt.enabled(RobotNetworkTables.Flag.HW_MOTOR_TUNING_WIDGETS)) {
            node.putDouble("currentLimitA", getCachedCurrentLimit());
            node.putBoolean("inverted", isCachedInverted());
            node.putBoolean("brakeMode", getCachedNeutralMode() == MotorNeutralMode.Brake);

            Encoder encoder = getEncoder();
            if (encoder != null) {
                encoder.networkTables(node.child("Encoder"));
            }
        }

        return node;
    }
}
