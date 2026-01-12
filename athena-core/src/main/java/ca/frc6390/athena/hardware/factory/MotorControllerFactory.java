package ca.frc6390.athena.hardware.factory;

import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerType;

/**
 * Service provider interface for creating motor controllers from a {@link MotorControllerConfig}.
 */
public interface MotorControllerFactory {
    boolean supports(MotorControllerType type);

    MotorController create(MotorControllerConfig config);
}
