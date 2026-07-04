package ca.frc6390.athena.hardware.ref;

import java.util.Locale;
import java.util.Objects;

import ca.frc6390.athena.api.hardware.AthenaEncoder;
import ca.frc6390.athena.api.hardware.EncoderId;
import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.hardware.encoder.EncoderSignalType;

/**
 * Reusable encoder declaration for robot constants.
 */
public record EncoderRef(
        EncoderSource source,
        EncoderSignalType signalType,
        boolean isInverted,
        double gearRatio,
        double conversion,
        double offset) {
    public static EncoderRef of(EncoderKind kind, int id) {
        return new EncoderRef(
                new EncoderSource.Standalone(kind, id, "rio"),
                EncoderSignalType.RELATIVE_POSITION,
                false,
                1.0,
                1.0,
                0.0);
    }

    public static EncoderRef of(EncoderId id) {
        Objects.requireNonNull(id, "id");
        return of(id.kind(), id.id()).canbus(id.canbus());
    }

    public static EncoderRef integrated(MotorRef motor) {
        return new EncoderRef(
                new EncoderSource.IntegratedMotor(Objects.requireNonNull(motor, "motor")),
                EncoderSignalType.RELATIVE_POSITION,
                false,
                1.0,
                1.0,
                0.0);
    }

    public EncoderRef {
        Objects.requireNonNull(source, "source");
        signalType = signalType == null ? EncoderSignalType.RELATIVE_POSITION : signalType;
    }

    public EncoderKind kind() {
        if (source instanceof EncoderSource.Standalone standalone) {
            return standalone.kind();
        }
        return AthenaEncoder.INTEGRATED_MOTOR;
    }

    public int id() {
        if (source instanceof EncoderSource.Standalone standalone) {
            return standalone.id();
        }
        return ((EncoderSource.IntegratedMotor) source).motor().id();
    }

    public String canbus() {
        if (source instanceof EncoderSource.Standalone standalone) {
            return standalone.canbus();
        }
        return ((EncoderSource.IntegratedMotor) source).motor().canbus();
    }

    public EncoderRef canbus(String canbus) {
        if (source instanceof EncoderSource.Standalone standalone) {
            return new EncoderRef(
                    new EncoderSource.Standalone(standalone.kind(), standalone.id(), canbus),
                    signalType,
                    isInverted,
                    gearRatio,
                    conversion,
                    offset);
        }
        return this;
    }

    public EncoderRef absolutePosition() {
        return signalType(EncoderSignalType.ABSOLUTE_POSITION);
    }

    public EncoderRef absolute() {
        return absolutePosition();
    }

    public EncoderRef relativePosition() {
        return signalType(EncoderSignalType.RELATIVE_POSITION);
    }

    public EncoderRef velocity() {
        return signalType(EncoderSignalType.VELOCITY);
    }

    public EncoderRef signalType(EncoderSignalType signalType) {
        return new EncoderRef(source, signalType, isInverted, gearRatio, conversion, offset);
    }

    public EncoderRef inverted() {
        return inverted(true);
    }

    public EncoderRef inverted(boolean inverted) {
        return new EncoderRef(source, signalType, inverted, gearRatio, conversion, offset);
    }

    public EncoderRef gearRatio(double gearRatio) {
        return new EncoderRef(source, signalType, isInverted, gearRatio, conversion, offset);
    }

    public EncoderRef gearRatio(GearRatioRef gearRatio) {
        Objects.requireNonNull(gearRatio, "gearRatio");
        return gearRatio(gearRatio.ratio());
    }

    public EncoderRef conversion(double conversion) {
        return new EncoderRef(source, signalType, isInverted, gearRatio, conversion, offset);
    }

    public EncoderRef offset(double offset) {
        return new EncoderRef(source, signalType, isInverted, gearRatio, conversion, offset);
    }

    public String defaultName() {
        return sanitize(kind().key()) + "_" + id();
    }

    private static String sanitize(String key) {
        return key.toLowerCase(Locale.ROOT).replace(':', '_').replace('-', '_');
    }

    public sealed interface EncoderSource permits EncoderSource.Standalone, EncoderSource.IntegratedMotor {
        record Standalone(EncoderKind kind, int id, String canbus) implements EncoderSource {
            public Standalone {
                Objects.requireNonNull(kind, "kind");
                canbus = canbus == null || canbus.isBlank() ? "rio" : canbus;
            }
        }

        record IntegratedMotor(MotorRef motor) implements EncoderSource {
            public IntegratedMotor {
                Objects.requireNonNull(motor, "motor");
            }
        }
    }
}
