package ca.frc6390.athena.hardware.examples;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderRegistry;
import ca.frc6390.athena.hardware.factory.HardwareFactories;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.ImuRegistry;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorRegistry;

/**
 * Examples for resolving registry keys into configs and creating hardware through factories.
 */
public final class HardwareFactoryExamples {
    private HardwareFactoryExamples() {}

    public static MotorControllerConfig motorConfig(String key, int id) {
        return MotorControllerConfig.create(MotorRegistry.get().motor(key), id);
    }

    public static EncoderConfig encoderConfig(String key, int id) {
        return EncoderConfig.create(EncoderRegistry.get().encoder(key), id);
    }

    public static ImuConfig imuConfig(String key, int id) {
        return ImuConfig.create(ImuRegistry.get().imu(key), id);
    }

    public static MotorController createMotor(MotorControllerConfig config) {
        return HardwareFactories.motor(config);
    }

    public static Encoder createEncoder(EncoderConfig config) {
        return HardwareFactories.encoder(config);
    }

    public static Imu createImu(ImuConfig config) {
        return HardwareFactories.imu(config);
    }
}
