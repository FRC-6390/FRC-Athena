package ca.frc6390.athena.vision.runtime;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Time;

/**
 * Timing, noise and loss model applied to simulated camera frames.
 *
 * <p>Frames are captured on a fixed grid at {@code fps}. Each frame is delivered {@code latency} (plus Gaussian
 * jitter) after capture and stamped with its capture time, so localization latency compensation sees the same
 * timestamps a real pipeline reports. The reported pose is the robot pose at the capture time plus Gaussian noise
 * whose standard deviation grows with the square of the average tag distance:</p>
 *
 * <pre>
 *   sigmaXY      = translationStdAt1m * d^2 * (singleTag ? singleTagMultiplier : 1)
 *   sigmaHeading = headingStdAt1m     * d^2 * (singleTag ? singleTagMultiplier : 1)
 * </pre>
 *
 * <p>Single-tag frames additionally flip to a mirrored pose (reflected across the tag plane) with
 * {@code ambiguityFlipProbability}, reporting a high ambiguity so ambiguity filters have something to reject.
 * Tags outside the camera's horizontal field of view or beyond {@code maxRange} are not seen.</p>
 *
 * <p>All randomness comes from {@code seed}, so a simulation with the same configuration and inputs produces the
 * same measurements.</p>
 *
 * @param fps frame rate; {@code +Infinity} delivers one frame per robot update
 * @param latency mean capture-to-delivery pipeline latency
 * @param jitter standard deviation of the per-frame latency
 * @param translationStdAt1m translation standard deviation of a multi-tag pose at 1 m average distance
 * @param headingStdAt1m heading standard deviation of a multi-tag pose at 1 m average distance
 * @param singleTagMultiplier factor applied to the noise of single-tag frames (a dimensionless gain)
 * @param dropProbability probability that a frame is lost
 * @param ambiguityFlipProbability probability that a single-tag frame reports a mirrored pose
 * @param maxRange largest tag distance that is visible
 * @param seed seed of the deterministic random stream
 */
public record VisionSimNoise(
        Frequency fps,
        Time latency,
        Time jitter,
        Distance translationStdAt1m,
        Angle headingStdAt1m,
        double singleTagMultiplier,
        double dropProbability,
        double ambiguityFlipProbability,
        Distance maxRange,
        long seed) {

    /** Default frame rate, used when no rate is configured. */
    public static final Frequency DEFAULT_FPS = Hertz.of(30.0);
    /** Default seed of the realistic model. */
    public static final long DEFAULT_SEED = 6390L;

    public VisionSimNoise {
        requireNonNull(fps, "fps");
        requireNonNull(latency, "latency");
        requireNonNull(jitter, "jitter");
        requireNonNull(translationStdAt1m, "translationStdAt1m");
        requireNonNull(headingStdAt1m, "headingStdAt1m");
        requireNonNull(maxRange, "maxRange");
        requireNonNegativeFinite(latency.in(Milliseconds), "latency");
        requireNonNegativeFinite(jitter.in(Milliseconds), "jitter");
        requireNonNegativeFinite(translationStdAt1m.in(Meters), "translationStdAt1m");
        requireNonNegativeFinite(headingStdAt1m.in(Degrees), "headingStdAt1m");
        requireNonNegativeFinite(singleTagMultiplier, "singleTagMultiplier");
        requireProbability(dropProbability, "dropProbability");
        requireProbability(ambiguityFlipProbability, "ambiguityFlipProbability");
        double framesPerSecond = fps.in(Hertz);
        if (Double.isNaN(framesPerSecond) || framesPerSecond <= 0.0) {
            throw new IllegalArgumentException("fps must be positive or +Infinity: " + framesPerSecond);
        }
        double range = maxRange.in(Meters);
        if (Double.isNaN(range) || range <= 0.0) {
            throw new IllegalArgumentException("maxRange must be positive or +Infinity: " + range);
        }
    }

    /**
     * Returns the default model: 30 fps, 25 +- 5 ms latency, 5 mm per meter squared of translation noise on
     * multi-tag frames, 0.1 deg per meter squared of heading noise, single-tag frames four times noisier, 2 % frame
     * drops, 5 % single-tag ambiguity flips, 8 m maximum range.
     *
     * @return realistic noise model
     */
    public static VisionSimNoise realistic() {
        return new VisionSimNoise(DEFAULT_FPS, Milliseconds.of(25.0), Milliseconds.of(5.0), Meters.of(0.005),
                Degrees.of(0.1), 4.0, 0.02, 0.05, Meters.of(8.0), DEFAULT_SEED);
    }

    /**
     * Returns an exact model for pipeline-only tests: a frame on every robot update, zero latency, exact poses, no
     * drops or flips and no range limit. Reported ambiguity is zero and no standard deviations are reported.
     *
     * @return perfect noise model
     */
    public static VisionSimNoise perfect() {
        return new VisionSimNoise(Hertz.of(Double.POSITIVE_INFINITY), Milliseconds.of(0.0), Milliseconds.of(0.0),
                Meters.of(0.0), Degrees.of(0.0), 1.0, 0.0, 0.0, Meters.of(Double.POSITIVE_INFINITY), 0L);
    }

    /**
     * Returns whether this model reproduces exact measurements (no timing, noise, loss or range limit).
     *
     * @return true when perfect
     */
    public boolean isPerfect() {
        return Double.isInfinite(fps.in(Hertz)) && latency.in(Milliseconds) == 0.0
                && jitter.in(Milliseconds) == 0.0 && translationStdAt1m.in(Meters) == 0.0
                && headingStdAt1m.in(Degrees) == 0.0 && dropProbability == 0.0
                && ambiguityFlipProbability == 0.0 && Double.isInfinite(maxRange.in(Meters));
    }

    /** Returns a copy with a frame rate, or {@code Hertz.of(+Infinity)} for every robot update. */
    public VisionSimNoise fps(Frequency framesPerSecond) {
        return new VisionSimNoise(framesPerSecond, latency, jitter, translationStdAt1m, headingStdAt1m,
                singleTagMultiplier, dropProbability, ambiguityFlipProbability, maxRange, seed);
    }

    /** Returns a copy with a mean capture-to-delivery latency. */
    public VisionSimNoise latency(Time mean) {
        return new VisionSimNoise(fps, mean, jitter, translationStdAt1m, headingStdAt1m,
                singleTagMultiplier, dropProbability, ambiguityFlipProbability, maxRange, seed);
    }

    /** Returns a copy with a latency standard deviation. */
    public VisionSimNoise jitter(Time stdDev) {
        return new VisionSimNoise(fps, latency, stdDev, translationStdAt1m, headingStdAt1m,
                singleTagMultiplier, dropProbability, ambiguityFlipProbability, maxRange, seed);
    }

    /** Returns a copy with the multi-tag translation standard deviation at 1 m. */
    public VisionSimNoise translationStdAt1m(Distance std) {
        return new VisionSimNoise(fps, latency, jitter, std, headingStdAt1m,
                singleTagMultiplier, dropProbability, ambiguityFlipProbability, maxRange, seed);
    }

    /** Returns a copy with the multi-tag heading standard deviation at 1 m. */
    public VisionSimNoise headingStdAt1m(Angle std) {
        return new VisionSimNoise(fps, latency, jitter, translationStdAt1m, std,
                singleTagMultiplier, dropProbability, ambiguityFlipProbability, maxRange, seed);
    }

    /** Returns a copy whose single-tag frames are this many times noisier than multi-tag frames. */
    public VisionSimNoise singleTagMultiplier(double multiplier) {
        return new VisionSimNoise(fps, latency, jitter, translationStdAt1m, headingStdAt1m,
                multiplier, dropProbability, ambiguityFlipProbability, maxRange, seed);
    }

    /** Returns a copy with the probability that a frame is lost. */
    public VisionSimNoise dropProbability(double probability) {
        return new VisionSimNoise(fps, latency, jitter, translationStdAt1m, headingStdAt1m,
                singleTagMultiplier, probability, ambiguityFlipProbability, maxRange, seed);
    }

    /** Returns a copy with the probability that a single-tag frame reports a mirrored pose. */
    public VisionSimNoise ambiguityFlipProbability(double probability) {
        return new VisionSimNoise(fps, latency, jitter, translationStdAt1m, headingStdAt1m,
                singleTagMultiplier, dropProbability, probability, maxRange, seed);
    }

    /** Returns a copy with the maximum visible tag distance. */
    public VisionSimNoise maxRange(Distance range) {
        return new VisionSimNoise(fps, latency, jitter, translationStdAt1m, headingStdAt1m,
                singleTagMultiplier, dropProbability, ambiguityFlipProbability, range, seed);
    }

    /** Returns a copy with a seed for the deterministic random stream. */
    public VisionSimNoise seed(long randomSeed) {
        return new VisionSimNoise(fps, latency, jitter, translationStdAt1m, headingStdAt1m,
                singleTagMultiplier, dropProbability, ambiguityFlipProbability, maxRange, randomSeed);
    }

    private static void requireNonNull(Object value, String name) {
        if (value == null) {
            throw new IllegalArgumentException(name + " must not be null");
        }
    }

    private static void requireNonNegativeFinite(double value, String name) {
        if (!Double.isFinite(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be finite and non-negative: " + value);
        }
    }

    private static void requireProbability(double value, String name) {
        if (!Double.isFinite(value) || value < 0.0 || value > 1.0) {
            throw new IllegalArgumentException(name + " must be in [0, 1]: " + value);
        }
    }
}
