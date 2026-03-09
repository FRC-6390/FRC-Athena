package ca.frc6390.athena.hardware.encoder;

import edu.wpi.first.wpilibj.Timer;

/**
 * Derives velocity from a position-like encoder signal.
 */
public final class DifferentiatedEncoder extends AbstractDerivedEncoder {
    private final DerivedEncoderInput input;
    private final SignalFilter filter;
    private double lastSample = Double.NaN;
    private double lastTimestampSeconds = Double.NaN;

    public DifferentiatedEncoder(
            EncoderConfig config,
            DerivedEncoderInput input,
            SignalFilter filter) {
        super(config, OutputKind.VELOCITY, false);
        this.input = input;
        this.filter = filter;
    }

    @Override
    public Encoder update() {
        double raw = input.read(true);
        double sample = raw;
        if (input.isWrappedAbsolute() && Double.isFinite(lastSample)) {
            sample = nearestEquivalent(raw, lastSample, input.wrapsEvery());
        }

        double nowSeconds = Timer.getFPGATimestamp();
        double velocity = 0.0;
        if (Double.isFinite(lastSample) && Double.isFinite(lastTimestampSeconds) && nowSeconds > lastTimestampSeconds) {
            velocity = (sample - lastSample) / (nowSeconds - lastTimestampSeconds);
        }
        if (filter != null) {
            velocity = filter.apply(velocity);
        }
        cachedValue = velocity;
        lastSample = sample;
        lastTimestampSeconds = nowSeconds;
        return this;
    }
}
