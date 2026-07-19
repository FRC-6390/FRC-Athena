package ca.frc6390.athena.hardware.signal;

/** Scalar mechanism input that retains its physical dependencies. */
@FunctionalInterface
public interface ScalarSignal extends FeedbackSignal {
    /** Returns the current scalar value. */
    double value();

    /** Returns this signal using the opposite sign convention. */
    default ScalarSignal inverted() {
        return inverted(true);
    }

    /** Selects whether this signal uses the opposite sign convention. */
    default ScalarSignal inverted(boolean inverted) {
        if (!inverted) return this;
        ScalarSignal source = this;
        return new ScalarSignal() {
            @Override public double value() { return -source.value(); }
            @Override public java.util.List<?> dependencies() { return source.dependencies(); }
        };
    }
}
