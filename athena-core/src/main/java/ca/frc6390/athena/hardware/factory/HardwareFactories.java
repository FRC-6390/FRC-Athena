package ca.frc6390.athena.hardware.factory;

import java.util.ServiceLoader;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;

/**
 * Convenience accessors that locate vendor-specific factories via {@link ServiceLoader}.
 */
public final class HardwareFactories {
    private static final ServiceLoader<MotorControllerFactory> MOTOR_FACTORIES =
            ServiceLoader.load(MotorControllerFactory.class);
    private static final ServiceLoader<EncoderFactory> ENCODER_FACTORIES =
            ServiceLoader.load(EncoderFactory.class);
    private static final ServiceLoader<ImuFactory> IMU_FACTORIES =
            ServiceLoader.load(ImuFactory.class);

    private HardwareFactories() {}

    public static MotorController motor(MotorControllerConfig config) {
        return MOTOR_FACTORIES.stream()
                .map(ServiceLoader.Provider::get)
                .filter(f -> f.supports(config.type))
                .findFirst()
                .orElseThrow(() -> new IllegalArgumentException(
                        "No motor factory for type: " + config.type.getKey()))
                .create(config);
    }

    public static Encoder encoder(EncoderConfig config) {
        return ENCODER_FACTORIES.stream()
                .map(ServiceLoader.Provider::get)
                .filter(f -> f.supports(config.type))
                .findFirst()
                .orElseThrow(() -> new IllegalArgumentException(
                        "No encoder factory for type: " + config.type.getKey()))
                .create(config);
    }

    public static Imu imu(ImuConfig config) {
        return IMU_FACTORIES.stream()
                .map(ServiceLoader.Provider::get)
                .filter(f -> f.supports(config.type))
                .findFirst()
                .orElseThrow(() -> new IllegalArgumentException(
                        "No IMU factory for type: " + config.type.getKey()))
                .create(config);
    }
}
