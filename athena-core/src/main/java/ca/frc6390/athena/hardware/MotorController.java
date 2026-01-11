package ca.frc6390.athena.hardware;

import edu.wpi.first.math.controller.PIDController;

/**
 * Vendor-agnostic motor controller interface for the new vendordep system.
 */
public interface MotorController {
    int getId();

    String getCanbus();

    MotorControllerType getType();

    void setSpeed(double percent);

    void setVoltage(double volts);

    void setCurrentLimit(double amps);

    void setPosition(double rotations);

    void setNeutralMode(MotorNeutralMode mode);

    void setPid(PIDController pid);

    boolean isConnected();

    double getTemperatureCelsius();

    Encoder getEncoder();
}
