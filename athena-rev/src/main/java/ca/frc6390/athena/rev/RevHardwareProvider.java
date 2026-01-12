package ca.frc6390.athena.rev;

import ca.frc6390.athena.rev.encoder.RevEncoderType;
import ca.frc6390.athena.rev.motor.RevMotorControllerType;
import ca.frc6390.athena.hardware.encoder.EncoderRegistry;
import ca.frc6390.athena.hardware.motor.MotorRegistry;

/**
 * Registers REV hardware types with the per-category registries.
 */
public class RevHardwareProvider implements MotorRegistry.Provider, EncoderRegistry.Provider {
    @Override
    public void register(MotorRegistry registry) {
        registry.add(RevMotorControllerType.SPARK_MAX_BRUSHED);
        registry.add(RevMotorControllerType.SPARK_MAX_BRUSHLESS);
        registry.add(RevMotorControllerType.SPARK_FLEX_BRUSHED);
        registry.add(RevMotorControllerType.SPARK_FLEX_BRUSHLESS);
    }

    @Override
    public void register(EncoderRegistry registry) {
        registry.add(RevEncoderType.SPARK_MAX);
        registry.add(RevEncoderType.SPARK_FLEX);
    }
}
