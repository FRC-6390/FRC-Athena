package ca.frc6390.athena.hardware.motor;

import java.util.HashMap;
import java.util.Map;
import java.util.Objects;
import java.util.ServiceLoader;

/**
 * Discovers vendor-specific motor controller types via {@link ServiceLoader}. Missing providers
 * surface clear guidance to add the appropriate athena-* module.
 */
public final class MotorRegistry {
    /** Implemented by vendor modules to register motor controller types. */
    public interface Provider {
        void register(MotorRegistry registry);
    }

    private static final MotorRegistry INSTANCE = new MotorRegistry();

    static {
        ServiceLoader.load(Provider.class).forEach(p -> p.register(INSTANCE));
    }

    private final Map<String, MotorControllerType> motors = new HashMap<>();

    private MotorRegistry() {}

    public static MotorRegistry get() {
        return INSTANCE;
    }

    public MotorRegistry add(MotorControllerType type) {
        motors.put(type.getKey(), type);
        return this;
    }

    public MotorControllerType motor(String key) {
        return Objects.requireNonNull(motors.get(key), missing(key));
    }

    private static String missing(String key) {
        return "Missing provider for motor '" + key
                + "'. Add the appropriate athena-* vendor module to dependencies.";
    }
}
