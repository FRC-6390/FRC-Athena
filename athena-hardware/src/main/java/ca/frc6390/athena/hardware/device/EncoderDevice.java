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
        return new EncoderDevice(
                new EncoderSource.Standalone(kind, id, "rio"),
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
            return standalone.id();
        }
        return motorSource().id();
    }

    /**
     * Returns this standalone encoder's roboRIO DIO channel.
     *
     * @return DIO channel
     */
    public int dioChannel() {
        if (source instanceof EncoderSource.Standalone standalone && standalone.canbus().equals("rio")) {
            return standalone.id();
        }
        throw new IllegalStateException("Encoder " + defaultName() + " is not a roboRIO DIO encoder.");
    }

    public String canbus() {
        if (source instanceof EncoderSource.Standalone standalone) {
            return standalone.canbus();
        }
        return motorSource().canbus();
    }

    public EncoderDevice canbus(String canbus) {
        if (source instanceof EncoderSource.Standalone standalone) {
            return new EncoderDevice(
                    new EncoderSource.Standalone(standalone.kind(), standalone.id(), canbus),
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

    public String defaultName() {
        return sanitize(kind().key()) + "_" + id();
    }

    private static String sanitize(String key) {
        return key.toLowerCase(Locale.ROOT).replace(':', '_').replace('-', '_');
    }

    public sealed interface EncoderSource
            permits EncoderSource.Standalone, EncoderSource.IntegratedMotor, EncoderSource.MotorAbsolute {
        record Standalone(EncoderKind kind, int id, String canbus) implements EncoderSource {
            public Standalone {
                Objects.requireNonNull(kind, "kind");
                canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
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
}
