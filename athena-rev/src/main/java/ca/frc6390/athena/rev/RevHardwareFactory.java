package ca.frc6390.athena.rev;

import ca.frc6390.athena.rev.encoder.RevEncoder;
import ca.frc6390.athena.rev.encoder.RevEncoderType;
import ca.frc6390.athena.rev.motor.RevMotorController;
import ca.frc6390.athena.rev.motor.RevMotorControllerType;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderType;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.factory.EncoderFactory;
import ca.frc6390.athena.hardware.factory.MotorControllerFactory;

/**
 * REV factory wiring for the new hardware registries.
 */
public class RevHardwareFactory implements MotorControllerFactory, EncoderFactory {
    @Override
    public boolean supports(MotorControllerType type) {
        return type instanceof RevMotorControllerType;
    }

    @Override
    public MotorController create(MotorControllerConfig config) {
        return RevMotorController.fromConfig(config);
    }

    @Override
    public boolean supports(EncoderType type) {
        return type instanceof RevEncoderType;
    }

    @Override
    public Encoder create(EncoderConfig config) {
        return RevEncoder.fromConfig(config);
    }
}
