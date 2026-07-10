package ca.frc6390.athena.hardware.device;

import java.util.Locale;
import java.util.Objects;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;

/**
 * Reusable encoder declaration for robot constants.
 */
public record EncoderDevice(
        EncoderSource source,
        boolean isInverted,
        double gearRatio,
        double conversion,
        double offset,
        EncoderUnit units) {
    public static EncoderDevice of(EncoderKind kind, int id) {
        return connected(kind, "rio", HardwarePort.can(id));
    }

    static EncoderDevice connected(EncoderKind kind, String bus, HardwarePort port) {
        return new EncoderDevice(
                new EncoderSource.Standalone(kind, bus, port),
                false,
                1.0,
                1.0,
                0.0,
                EncoderUnit.RAW);
    }

    static EncoderDevice integratedMotor(MotorDevice motor) {
        return new EncoderDevice(
                new EncoderSource.IntegratedMotor(Objects.requireNonNull(motor, "motor")),
                false,
                1.0,
                1.0,
                0.0,
                EncoderUnit.RAW);
    }

    static EncoderDevice motorAbsolute(MotorDevice motor) {
        return new EncoderDevice(
                new EncoderSource.MotorAbsolute(Objects.requireNonNull(motor, "motor")),
                false,
                1.0,
                1.0,
                0.0,
                EncoderUnit.RAW);
    }

    public EncoderDevice {
        Objects.requireNonNull(source, "source");
        if (!Double.isFinite(gearRatio) || gearRatio <= 0.0) {
            throw new IllegalArgumentException("Encoder gear ratio must be positive.");
        }
        if (!Double.isFinite(conversion) || conversion <= 0.0) {
            throw new IllegalArgumentException("Encoder conversion must be positive.");
        }
        if (!Double.isFinite(offset)) {
            throw new IllegalArgumentException("Encoder offset must be finite.");
        }
        units = units == null ? EncoderUnit.RAW : units;
    }

    public EncoderKind kind() {
        if (source instanceof EncoderSource.Standalone standalone) {
            return standalone.kind();
        }
        return EncoderKinds.INTEGRATED_MOTOR;
    }

    public int id() {
        if (source instanceof EncoderSource.Standalone standalone) {
            return requireCan(standalone).id();
        }
        return motorSource().id();
    }

    public HardwarePort port() {
        if (source instanceof EncoderSource.Standalone standalone) {
            return standalone.port();
        }
        throw new IllegalStateException("Motor-integrated encoder " + defaultName() + " does not have a standalone port.");
    }

    public String bus() {
        if (source instanceof EncoderSource.Standalone standalone) {
            return standalone.bus();
        }
        return motorSource().canbus();
    }

    public String canbus() {
        if (source instanceof EncoderSource.Standalone standalone) {
            requireCan(standalone);
            return standalone.bus();
        }
        return motorSource().canbus();
    }

    public EncoderDevice canbus(String canbus) {
        if (source instanceof EncoderSource.Standalone standalone) {
            requireCan(standalone);
            return new EncoderDevice(
                    new EncoderSource.Standalone(standalone.kind(), canbus, standalone.port()),
                    isInverted,
                    gearRatio,
                    conversion,
                    offset,
                    units);
        }
        return this;
    }

    public EncoderDevice inverted() {
        return inverted(true);
    }

    public EncoderDevice inverted(boolean inverted) {
        return new EncoderDevice(source, inverted, gearRatio, conversion, offset, units);
    }

    public EncoderDevice gearRatio(double gearRatio) {
        return new EncoderDevice(source, isInverted, gearRatio, conversion, offset, units);
    }

    public EncoderDevice gearRatio(GearRatio gearRatio) {
        Objects.requireNonNull(gearRatio, "gearRatio");
        return gearRatio(gearRatio.ratio());
    }

    public EncoderDevice conversion(double conversion) {
        return new EncoderDevice(source, isInverted, gearRatio, conversion, offset, units);
    }

    public EncoderDevice wheelDiameterMeters(double diameterMeters) {
        if (!Double.isFinite(diameterMeters) || diameterMeters <= 0.0) {
            throw new IllegalArgumentException("Wheel diameter must be positive.");
        }
        return conversion(Math.PI * diameterMeters);
    }

    public EncoderDevice wheelDiameterInches(double diameterInches) {
        return wheelDiameterMeters(diameterInches * 0.0254);
    }

    public EncoderDevice units(EncoderUnit units) {
        return new EncoderDevice(source, isInverted, gearRatio, conversion, offset, units);
    }

    public EncoderDevice offset(double offset) {
        return new EncoderDevice(source, isInverted, gearRatio, conversion, offset, units);
    }

    /**
     * Creates an action that sets this encoder's relative position.
     *
     * @param position relative position in this declaration's configured units
     * @param <T> mechanism action type
     * @return position-setting action
     */
    public <T> T setPosition(double position) {
        if (!Double.isFinite(position)) {
            throw new IllegalArgumentException("Encoder position must be finite.");
        }
        return action("setPosition", new Class<?>[] {EncoderDevice.class, double.class}, this, position);
    }

    /**
     * Converts raw sensor rotations into configured mechanism position.
     *
     * @param rotations raw sensor rotations
     * @return configured position
     */
    public double positionFromRotations(double rotations) {
        double directed = isInverted ? -rotations : rotations;
        return (directed - offset) * gearRatio * conversion;
    }

    /**
     * Converts raw sensor velocity into configured mechanism velocity.
     *
     * @param rotationsPerSecond raw sensor rotations per second
     * @return configured velocity
     */
    public double velocityFromRotationsPerSecond(double rotationsPerSecond) {
        double directed = isInverted ? -rotationsPerSecond : rotationsPerSecond;
        return directed * gearRatio * conversion;
    }

    /**
     * Converts configured mechanism position into raw sensor rotations.
     *
     * @param position configured position
     * @return raw sensor rotations
     */
    public double rotationsFromPosition(double position) {
        if (!Double.isFinite(position)) {
            throw new IllegalArgumentException("Encoder position must be finite.");
        }
        double directed = position / (gearRatio * conversion) + offset;
        return isInverted ? -directed : directed;
    }

    public String defaultName() {
        if (source instanceof EncoderSource.Standalone standalone) {
            return sanitize(kind().key()) + "_" + sanitize(standalone.port().identity()) + "_"
                    + standalone.port().primaryAddress();
        }
        return sanitize(kind().key()) + "_" + motorSource().id();
    }

    private static String sanitize(String key) {
        return key.toLowerCase(Locale.ROOT).replace(':', '_').replace('-', '_');
    }

    @SuppressWarnings("unchecked")
    private static <T> T action(String method, Class<?>[] parameterTypes, Object... args) {
        try {
            Class<?> actions = Class.forName("ca.frc6390.athena.mechanism.core.Actions");
            var factory = actions.getDeclaredMethod(method, parameterTypes);
            if (!factory.canAccess(null)) {
                factory.setAccessible(true);
            }
            return (T) factory.invoke(null, args);
        } catch (ReflectiveOperationException exception) {
            throw new IllegalStateException("Encoder action factories require athena-mechanisms on the classpath.", exception);
        }
    }

    public sealed interface EncoderSource
            permits EncoderSource.Standalone, EncoderSource.IntegratedMotor, EncoderSource.MotorAbsolute {
        record Standalone(EncoderKind kind, String bus, HardwarePort port) implements EncoderSource {
            public Standalone {
                Objects.requireNonNull(kind, "kind");
                bus = bus == null || bus.isBlank() ? "rio" : bus;
                Objects.requireNonNull(port, "port");
            }
        }

        record IntegratedMotor(MotorDevice motor) implements EncoderSource {
            public IntegratedMotor {
                Objects.requireNonNull(motor, "motor");
            }
        }

        record MotorAbsolute(MotorDevice motor) implements EncoderSource {
            public MotorAbsolute {
                Objects.requireNonNull(motor, "motor");
            }
        }
    }

    private MotorDevice motorSource() {
        if (source instanceof EncoderSource.IntegratedMotor integrated) {
            return integrated.motor();
        }
        return ((EncoderSource.MotorAbsolute) source).motor();
    }

    private static HardwarePort.Can requireCan(EncoderSource.Standalone standalone) {
        if (standalone.port() instanceof HardwarePort.Can can) {
            return can;
        }
        throw new IllegalStateException("Encoder " + standalone.kind().key() + " is not connected over CAN.");
    }
}
