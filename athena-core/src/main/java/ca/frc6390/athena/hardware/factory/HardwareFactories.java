package ca.frc6390.athena.hardware.factory;

import java.util.Map;
import java.util.ServiceLoader;
import java.util.concurrent.ConcurrentHashMap;

import ca.frc6390.athena.hardware.encoder.Encoder;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderType;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.imu.ImuType;
import ca.frc6390.athena.hardware.motor.MotorController;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerType;

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
    private static final Map<String, MotorControllerFactory> MOTOR_FACTORY_CACHE = new ConcurrentHashMap<>();
    private static final Map<String, EncoderFactory> ENCODER_FACTORY_CACHE = new ConcurrentHashMap<>();
    private static final Map<String, ImuFactory> IMU_FACTORY_CACHE = new ConcurrentHashMap<>();

    private HardwareFactories() {}

    public static MotorController motor(MotorControllerConfig config) {
        if (config == null || config.type() == null) {
            throw new IllegalArgumentException("Motor controller config type is required.");
        }
        MotorControllerFactory factory = resolveMotorFactory(config.type());
        MotorController raw = factory.create(config);
        return ca.frc6390.athena.hardware.motor.MotorControllerAdapter.wrap(raw, config);
    }

    public static Encoder encoder(EncoderConfig config) {
        if (config == null || config.type() == null) {
            throw new IllegalArgumentException("Encoder config type is required.");
        }
        EncoderFactory factory = resolveEncoderFactory(config.type());
        Encoder raw = factory.create(config);
        return ca.frc6390.athena.hardware.encoder.EncoderAdapter.wrap(raw, config);
    }

    public static Imu imu(ImuConfig config) {
        if (config == null || config.type() == null) {
            throw new IllegalArgumentException("IMU config type is required.");
        }
        ImuFactory factory = resolveImuFactory(config.type());
        Imu raw = factory.create(config);
        return ca.frc6390.athena.hardware.imu.ImuAdapter.wrap(raw, config);
    }

    private static MotorControllerFactory resolveMotorFactory(MotorControllerType type) {
        String key = type.getKey();
        if (key == null || key.isBlank()) {
            throw new IllegalArgumentException("Motor controller config type key is required.");
        }
        return MOTOR_FACTORY_CACHE.computeIfAbsent(key, __ -> {
            synchronized (MOTOR_FACTORIES) {
                return MOTOR_FACTORIES.stream()
                        .map(ServiceLoader.Provider::get)
                        .filter(factory -> factory.supports(type))
                        .findFirst()
                        .orElseThrow(() -> new IllegalArgumentException(
                                "No motor factory for type: " + key));
            }
        });
    }

    private static EncoderFactory resolveEncoderFactory(EncoderType type) {
        String key = type.getKey();
        if (key == null || key.isBlank()) {
            throw new IllegalArgumentException("Encoder config type key is required.");
        }
        return ENCODER_FACTORY_CACHE.computeIfAbsent(key, __ -> {
            synchronized (ENCODER_FACTORIES) {
                return ENCODER_FACTORIES.stream()
                        .map(ServiceLoader.Provider::get)
                        .filter(factory -> factory.supports(type))
                        .findFirst()
                        .orElseThrow(() -> new IllegalArgumentException(
                                "No encoder factory for type: " + key));
            }
        });
    }

    private static ImuFactory resolveImuFactory(ImuType type) {
        String key = type.getKey();
        if (key == null || key.isBlank()) {
            throw new IllegalArgumentException("IMU config type key is required.");
        }
        return IMU_FACTORY_CACHE.computeIfAbsent(key, __ -> {
            synchronized (IMU_FACTORIES) {
                return IMU_FACTORIES.stream()
                        .map(ServiceLoader.Provider::get)
                        .filter(factory -> factory.supports(type))
                        .findFirst()
                        .orElseThrow(() -> new IllegalArgumentException(
                                "No IMU factory for type: " + key));
            }
        });
    }
}
