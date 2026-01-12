package ca.frc6390.athena.ctre;

import ca.frc6390.athena.ctre.encoder.CtreEncoder;
import ca.frc6390.athena.ctre.encoder.CtreEncoderType;
import ca.frc6390.athena.ctre.imu.CtreImu;
import ca.frc6390.athena.ctre.imu.CtreImuType;
import ca.frc6390.athena.ctre.motor.CtreMotorController;
import ca.frc6390.athena.ctre.motor.CtreMotorControllerType;
import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderType;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.ImuType;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerType;
import ca.frc6390.athena.hardware.factory.EncoderFactory;
import ca.frc6390.athena.hardware.factory.ImuFactory;
import ca.frc6390.athena.hardware.factory.MotorControllerFactory;

/**
 * CTRE factory wiring for the new hardware registries.
 */
public class CtreHardwareFactory implements MotorControllerFactory, EncoderFactory, ImuFactory {
    @Override
    public boolean supports(MotorControllerType type) {
        return type instanceof CtreMotorControllerType;
    }

    @Override
    public MotorController create(MotorControllerConfig config) {
        return CtreMotorController.fromConfig(config);
    }

    @Override
    public boolean supports(EncoderType type) {
        return type instanceof CtreEncoderType;
    }

    @Override
    public Encoder create(EncoderConfig config) {
        return CtreEncoder.fromConfig(config);
    }

    @Override
    public boolean supports(ImuType type) {
        return type instanceof CtreImuType;
    }

    @Override
    public Imu create(ImuConfig config) {
        return CtreImu.fromConfig(config);
    }
}
