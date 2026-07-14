package ca.frc6390.athena.hardware.backend;

import java.util.ArrayList;
import java.util.Iterator;
import java.util.List;
import java.util.Optional;
import java.util.ServiceConfigurationError;
import java.util.ServiceLoader;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.MotorKind;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.ImuDevice;

/**
 * Registry of installed hardware backends.
 */
public final class BackendRegistry {
    private static volatile BackendRegistry global;

    private final List<MotorBackend> motorBackends;
    private final List<EncoderBackend> encoderBackends;
    private final List<ImuBackend> imuBackends;
    private final List<String> discoveryFailures;

    private BackendRegistry(
            List<MotorBackend> motorBackends,
            List<EncoderBackend> encoderBackends,
            List<ImuBackend> imuBackends,
            List<String> discoveryFailures) {
        this.motorBackends = List.copyOf(motorBackends);
        this.encoderBackends = List.copyOf(encoderBackends);
        this.imuBackends = List.copyOf(imuBackends);
        this.discoveryFailures = List.copyOf(discoveryFailures);
    }

    /**
     * Discovers backends from {@link ServiceLoader}.
     *
     * @return discovered registry
     */
    public static BackendRegistry discover() {
        List<MotorBackend> motors = new ArrayList<>();
        List<EncoderBackend> encoders = new ArrayList<>();
        List<ImuBackend> imus = new ArrayList<>();
        List<String> failures = new ArrayList<>();
        load(MotorBackend.class, motors, failures);
        load(EncoderBackend.class, encoders, failures);
        load(ImuBackend.class, imus, failures);
        return new BackendRegistry(motors, encoders, imus, failures);
    }

    private static <T> void load(Class<T> type, List<T> services, List<String> failures) {
        Iterator<T> iterator = ServiceLoader.load(type).iterator();
        while (true) {
            try {
                if (!iterator.hasNext()) {
                    return;
                }
                services.add(iterator.next());
            } catch (ServiceConfigurationError error) {
                failures.add(type.getSimpleName() + ": " + describe(error));
            }
        }
    }

    private static String describe(Throwable error) {
        Throwable cause = error;
        while (cause.getCause() != null) {
            cause = cause.getCause();
        }
        String provider = error.getMessage();
        String missing = cause.getMessage();
        String detail = cause.getClass().getSimpleName()
                + (missing == null || missing.isBlank() ? "" : ": " + missing);
        return provider == null || provider.isBlank() ? detail : provider + " (" + detail + ")";
    }

    /**
     * Creates a registry for tests and explicit contexts.
     *
     * @param motorBackends motor backends
     * @return registry
     */
    public static BackendRegistry of(MotorBackend... motorBackends) {
        if (motorBackends == null) {
            return of(List.of(), List.of(), List.of());
        }
        return of(List.of(motorBackends), List.of(), List.of());
    }

    /**
     * Creates a registry from explicit backend lists.
     *
     * @param motorBackends motor backends
     * @param encoderBackends encoder backends
     * @param imuBackends IMU backends
     * @return registry
     */
    public static BackendRegistry of(
            List<MotorBackend> motorBackends,
            List<EncoderBackend> encoderBackends,
            List<ImuBackend> imuBackends) {
        List<MotorBackend> motors = new ArrayList<>();
        if (motorBackends != null) {
            for (MotorBackend backend : motorBackends) {
                if (backend != null) {
                    motors.add(backend);
                }
            }
        }
        List<EncoderBackend> encoders = new ArrayList<>();
        if (encoderBackends != null) {
            for (EncoderBackend backend : encoderBackends) {
                if (backend != null) {
                    encoders.add(backend);
                }
            }
        }
        List<ImuBackend> imus = new ArrayList<>();
        if (imuBackends != null) {
            for (ImuBackend backend : imuBackends) {
                if (backend != null) {
                    imus.add(backend);
                }
            }
        }
        return new BackendRegistry(motors, encoders, imus, List.of());
    }

    public List<String> discoveryFailures() {
        return discoveryFailures;
    }

    public String missingBackendMessage(String deviceType, String kind) {
        String failures = discoveryFailures.isEmpty()
                ? "No installed backend advertises support for this device kind."
                : "Vendor backend loading failed: " + String.join("; ", discoveryFailures);
        return "Athena cannot start " + deviceType + " '" + kind + "'. " + failures
                + " Add the vendor's WPILib vendordep and redeploy the robot code.";
    }

    /**
     * Returns the global discovered registry.
     *
     * @return global registry
     */
    public static BackendRegistry global() {
        BackendRegistry registry = global;
        if (registry == null) {
            synchronized (BackendRegistry.class) {
                registry = global;
                if (registry == null) {
                    registry = discover();
                    global = registry;
                }
            }
        }
        return registry;
    }

    /**
     * Replaces the global registry. Passing null clears the override so the next global lookup discovers lazily.
     *
     * @param registry new registry
     */
    public static void setGlobal(BackendRegistry registry) {
        global = registry;
    }

    /**
     * Finds a motor backend.
     *
     * @param kind motor kind
     * @return backend if installed
     */
    public Optional<MotorBackend> motorBackendFor(MotorKind kind) {
        return motorBackends.stream()
                .filter(backend -> backend.supports(kind))
                .findFirst();
    }

    /**
     * Finds an encoder backend.
     *
     * @param kind encoder kind
     * @return backend if installed
     */
    public Optional<EncoderBackend> encoderBackendFor(EncoderKind kind) {
        return encoderBackends.stream()
                .filter(backend -> backend.supports(kind))
                .findFirst();
    }

    /**
     * Finds an encoder backend for a complete physical declaration.
     *
     * @param device encoder declaration
     * @return backend if installed
     */
    public Optional<EncoderBackend> encoderBackendFor(EncoderDevice device) {
        return encoderBackends.stream()
                .filter(backend -> backend.supports(device))
                .findFirst();
    }

    /**
     * Finds an IMU backend.
     *
     * @param kind IMU kind
     * @return backend if installed
     */
    public Optional<ImuBackend> imuBackendFor(ImuKind kind) {
        return imuBackends.stream()
                .filter(backend -> backend.supports(kind))
                .findFirst();
    }

    /**
     * Finds an IMU backend for a complete physical declaration.
     *
     * @param device IMU declaration
     * @return backend if installed
     */
    public Optional<ImuBackend> imuBackendFor(ImuDevice device) {
        return imuBackends.stream()
                .filter(backend -> backend.supports(device))
                .findFirst();
    }
}
