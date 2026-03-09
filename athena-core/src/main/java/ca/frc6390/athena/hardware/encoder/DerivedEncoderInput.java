package ca.frc6390.athena.hardware.encoder;

import ca.frc6390.athena.mechanisms.MechanismEncoderSource;
import ca.frc6390.athena.mechanisms.MechanismEncoderUnit;
import java.util.Objects;

/**
 * Resolved encoder input selection for a derived encoder source.
 */
public record DerivedEncoderInput(
        MechanismEncoderSource source,
        EncoderSignalType signal) {

    public DerivedEncoderInput {
        Objects.requireNonNull(source, "source");
        Objects.requireNonNull(signal, "signal");
        if (source.device() == null) {
            throw new IllegalArgumentException("derived encoder input source must have a device");
        }
    }

    public double read(boolean poll) {
        Encoder device = source.device();
        if (poll) {
            device.update();
        }
        return switch (signal) {
            case POSITION -> device.getPosition();
            case VELOCITY -> device.getVelocity();
            case ABSOLUTE -> device.getAbsolutePosition();
        };
    }

    public MechanismEncoderUnit unit() {
        return source.unit();
    }

    public double wrapsEvery() {
        return signal == EncoderSignalType.ABSOLUTE ? source.wrapsEvery() : Double.NaN;
    }

    public boolean isWrappedAbsolute() {
        return signal == EncoderSignalType.ABSOLUTE && source.supportsWrappedAbsolute();
    }
}
