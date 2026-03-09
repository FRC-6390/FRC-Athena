package ca.frc6390.athena.hardware.encoder;

/**
 * Filters a single encoder signal channel.
 */
public final class FilteredEncoder extends AbstractDerivedEncoder {
    private final DerivedEncoderInput input;
    private final SignalFilter filter;
    private double lastUnwrappedInput = Double.NaN;

    public FilteredEncoder(
            EncoderConfig config,
            DerivedEncoderInput input,
            SignalFilter filter) {
        super(
                config,
                input.signal() == EncoderSignalType.VELOCITY ? OutputKind.VELOCITY : OutputKind.POSITION,
                input.signal() == EncoderSignalType.ABSOLUTE);
        this.input = input;
        this.filter = filter;
    }

    @Override
    public Encoder update() {
        double raw = input.read(true);
        double working = raw;
        if (input.isWrappedAbsolute()) {
            double span = input.wrapsEvery();
            if (Double.isFinite(lastUnwrappedInput)) {
                working = nearestEquivalent(raw, lastUnwrappedInput, span);
            }
            lastUnwrappedInput = working;
        }
        double filtered = filter.apply(working);
        if (input.isWrappedAbsolute()) {
            filtered = nearestEquivalent(filtered, raw, input.wrapsEvery());
        }
        cachedValue = filtered;
        return this;
    }
}
