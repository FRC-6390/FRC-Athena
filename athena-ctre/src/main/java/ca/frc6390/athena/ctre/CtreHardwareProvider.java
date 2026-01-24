package ca.frc6390.athena.ctre;

import ca.frc6390.athena.ctre.encoder.CtreEncoderType;
import ca.frc6390.athena.ctre.imu.CtreImuType;
import ca.frc6390.athena.ctre.motor.CtreMotorControllerType;
import ca.frc6390.athena.hardware.encoder.EncoderRegistry;
import ca.frc6390.athena.hardware.imu.ImuRegistry;
import ca.frc6390.athena.hardware.motor.MotorRegistry;

/**
 * Registers CTRE hardware types with the per-category registries.
 */
public class CtreHardwareProvider implements MotorRegistry.Provider, EncoderRegistry.Provider, ImuRegistry.Provider {
    @Override
    public void register(MotorRegistry registry) {
        registry.add(CtreMotorControllerType.TALON_FX);
    }

    @Override
    public void register(EncoderRegistry registry) {
        registry.add(CtreEncoderType.CANCODER);
        registry.add(CtreEncoderType.TALON_FX);
    }

    @Override
    public void register(ImuRegistry registry) {
        registry.add(CtreImuType.PIGEON2);
    }
}
