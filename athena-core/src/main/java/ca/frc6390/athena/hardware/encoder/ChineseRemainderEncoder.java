package ca.frc6390.athena.hardware.encoder;

import ca.frc6390.athena.mechanisms.MechanismEncoderSource;
import edu.wpi.first.math.MathUtil;
import java.util.List;

/**
 * Fuses multiple wrapped absolute encoder sources into a bounded absolute mechanism position.
 */
public class ChineseRemainderEncoder implements Encoder {
    public record Input(MechanismEncoderSource source, int modulus) {
        public Input {
            if (source == null || source.device() == null) {
                throw new IllegalArgumentException("CRT input source is required.");
            }
            if (modulus <= 0) {
                throw new IllegalArgumentException("CRT modulus must be > 0.");
            }
        }
    }

    private final EncoderConfig config;
    private final List<Input> inputs;
    private final double validMin;
    private final double validMax;
    private double cachedPosition;
    private double cachedVelocity;
    private double lastUpdateSeconds = Double.NaN;

    public ChineseRemainderEncoder(
            EncoderConfig config,
            List<Input> inputs,
            double validMin,
            double validMax) {
        this.config = config != null ? config : EncoderConfig.create();
        this.inputs = List.copyOf(inputs);
        this.validMin = validMin;
        this.validMax = validMax;
    }

    @Override
    public double getPosition() {
        return cachedPosition;
    }

    @Override
    public double getVelocity() {
        return cachedVelocity;
    }

    @Override
    public double getAbsolutePosition() {
        return cachedPosition;
    }

    @Override
    public void setPosition(double position) {
        cachedPosition = position;
    }

    @Override
    public void setInverted(boolean inverted) {
    }

    @Override
    public void setConversion(double conversion) {
    }

    @Override
    public void setOffset(double offset) {
    }

    @Override
    public Encoder update() {
        for (Input input : inputs) {
            input.source().device().update();
        }
        double position = solvePosition();
        double nowSeconds = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
        if (Double.isFinite(lastUpdateSeconds) && nowSeconds > lastUpdateSeconds) {
            cachedVelocity = (position - cachedPosition) / (nowSeconds - lastUpdateSeconds);
        } else {
            cachedVelocity = 0.0;
        }
        cachedPosition = position;
        lastUpdateSeconds = nowSeconds;
        return this;
    }

    @Override
    public EncoderConfig getConfig() {
        return config;
    }

    private double solvePosition() {
        if (inputs.isEmpty()) {
            return 0.0;
        }
        Input anchor = inputs.get(0);
        double anchorSpan = anchor.source().wrapsEvery();
        double anchorReading = normalize(anchor.source().device().getAbsolutePosition(), anchorSpan);
        int candidates = Math.max(anchor.modulus(), 1);
        double best = clampToValid(anchorReading);
        double bestError = Double.POSITIVE_INFINITY;
        for (int k = 0; k < candidates; k++) {
            double candidate = anchorReading + (k * anchorSpan);
            if (!withinValid(candidate)) {
                continue;
            }
            double error = totalWrappedError(candidate);
            if (error < bestError) {
                best = candidate;
                bestError = error;
            }
        }
        if (Double.isFinite(bestError)) {
            return best;
        }
        return clampToValid(anchorReading);
    }

    private double totalWrappedError(double candidate) {
        double error = 0.0;
        for (Input input : inputs) {
            double span = input.source().wrapsEvery();
            double reading = normalize(input.source().device().getAbsolutePosition(), span);
            double expected = normalize(candidate, span);
            error += Math.abs(MathUtil.inputModulus(expected - reading, -span / 2.0, span / 2.0));
        }
        return error;
    }

    private boolean withinValid(double value) {
        if (!Double.isFinite(validMin) || !Double.isFinite(validMax) || validMax <= validMin) {
            return true;
        }
        return value >= validMin && value <= validMax;
    }

    private double clampToValid(double value) {
        if (!Double.isFinite(validMin) || !Double.isFinite(validMax) || validMax <= validMin) {
            return value;
        }
        return MathUtil.clamp(value, validMin, validMax);
    }

    private static double normalize(double value, double span) {
        if (!Double.isFinite(span) || span <= 0.0) {
            return value;
        }
        return MathUtil.inputModulus(value, 0.0, span);
    }
}
