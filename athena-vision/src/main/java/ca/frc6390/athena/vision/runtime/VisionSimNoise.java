package ca.frc6390.athena.vision.runtime;

/**
 * Timing, noise and loss model applied to simulated camera frames.
 *
 * <p>Frames are captured on a fixed grid at {@code fps}. Each frame is delivered {@code latencyMs} (plus Gaussian
 * jitter) after capture and stamped with its capture time, so localization latency compensation sees the same
 * timestamps a real pipeline reports. The reported pose is the robot pose at the capture time plus Gaussian noise
 * whose standard deviation grows with the square of the average tag distance:</p>
 *
 * <pre>
 *   sigmaXY      = translationStdAt1m * d^2 * (singleTag ? singleTagMultiplier : 1)
 *   sigmaHeading = headingStdDegAt1m  * d^2 * (singleTag ? singleTagMultiplier : 1)  (degrees)
 * </pre>
 *
 * <p>Single-tag frames additionally flip to a mirrored pose (reflected across the tag plane) with
 * {@code ambiguityFlipProbability}, reporting a high ambiguity so ambiguity filters have something to reject.
 * Tags outside the camera's horizontal field of view or beyond {@code maxRangeMeters} are not seen.</p>
 *
 * <p>All randomness comes from {@code seed}, so a simulation with the same configuration and inputs produces the
 * same measurements.</p>
 *
 * @param fps frame rate in frames per second; {@code +Infinity} delivers one frame per robot update
 * @param latencyMs mean capture-to-delivery pipeline latency in milliseconds
 * @param jitterMs standard deviation of the per-frame latency in milliseconds
 * @param translationStdAt1m translation standard deviation in meters of a multi-tag pose at 1 m average distance
 * @param headingStdDegAt1m heading standard deviation in degrees of a multi-tag pose at 1 m average distance
 * @param singleTagMultiplier factor applied to the noise of single-tag frames
 * @param dropProbability probability that a frame is lost
 * @param ambiguityFlipProbability probability that a single-tag frame reports a mirrored pose
 * @param maxRangeMeters largest tag distance that is visible
 * @param seed seed of the deterministic random stream
 */
public record VisionSimNoise(
        double fps,
        double latencyMs,
        double jitterMs,
        double translationStdAt1m,
        double headingStdDegAt1m,
        double singleTagMultiplier,
        double dropProbability,
        double ambiguityFlipProbability,
        double maxRangeMeters,
        long seed) {

    /** Default frame rate, used when no rate is configured. */
    public static final double DEFAULT_FPS = 30.0;
    /** Default seed of the realistic model. */
    public static final long DEFAULT_SEED = 6390L;

    public VisionSimNoise {
        requireNonNegativeFinite(latencyMs, "latencyMs");
        requireNonNegativeFinite(jitterMs, "jitterMs");
        requireNonNegativeFinite(translationStdAt1m, "translationStdAt1m");
        requireNonNegativeFinite(headingStdDegAt1m, "headingStdDegAt1m");
        requireNonNegativeFinite(singleTagMultiplier, "singleTagMultiplier");
        requireProbability(dropProbability, "dropProbability");
        requireProbability(ambiguityFlipProbability, "ambiguityFlipProbability");
        if (Double.isNaN(fps) || fps <= 0.0) {
            throw new IllegalArgumentException("fps must be positive or +Infinity: " + fps);
        }
        if (Double.isNaN(maxRangeMeters) || maxRangeMeters <= 0.0) {
            throw new IllegalArgumentException("maxRangeMeters must be positive or +Infinity: " + maxRangeMeters);
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
        return new VisionSimNoise(DEFAULT_FPS, 25.0, 5.0, 0.005, 0.1, 4.0, 0.02, 0.05, 8.0, DEFAULT_SEED);
    }

    /**
     * Returns an exact model for pipeline-only tests: a frame on every robot update, zero latency, exact poses, no
     * drops or flips and no range limit. Reported ambiguity is zero and no standard deviations are reported.
     *
     * @return perfect noise model
     */
    public static VisionSimNoise perfect() {
        return new VisionSimNoise(Double.POSITIVE_INFINITY, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                Double.POSITIVE_INFINITY, 0L);
    }

    /**
     * Returns whether this model reproduces exact measurements (no timing, noise, loss or range limit).
     *
     * @return true when perfect
     */
    public boolean isPerfect() {
        return Double.isInfinite(fps) && latencyMs == 0.0 && jitterMs == 0.0 && translationStdAt1m == 0.0
                && headingStdDegAt1m == 0.0 && dropProbability == 0.0 && ambiguityFlipProbability == 0.0
                && Double.isInfinite(maxRangeMeters);
    }

    /** Returns a copy with a frame rate in frames per second, or {@code +Infinity} for every robot update. */
    public VisionSimNoise fps(double framesPerSecond) {
        return new VisionSimNoise(framesPerSecond, latencyMs, jitterMs, translationStdAt1m, headingStdDegAt1m,
                singleTagMultiplier, dropProbability, ambiguityFlipProbability, maxRangeMeters, seed);
    }

    /** Returns a copy with a mean capture-to-delivery latency in milliseconds. */
    public VisionSimNoise latencyMs(double meanMs) {
        return new VisionSimNoise(fps, meanMs, jitterMs, translationStdAt1m, headingStdDegAt1m,
                singleTagMultiplier, dropProbability, ambiguityFlipProbability, maxRangeMeters, seed);
    }

    /** Returns a copy with a latency standard deviation in milliseconds. */
    public VisionSimNoise jitterMs(double stdDevMs) {
        return new VisionSimNoise(fps, latencyMs, stdDevMs, translationStdAt1m, headingStdDegAt1m,
                singleTagMultiplier, dropProbability, ambiguityFlipProbability, maxRangeMeters, seed);
    }

    /** Returns a copy with the multi-tag translation standard deviation at 1 m, in meters. */
    public VisionSimNoise translationStdAt1m(double meters) {
        return new VisionSimNoise(fps, latencyMs, jitterMs, meters, headingStdDegAt1m,
                singleTagMultiplier, dropProbability, ambiguityFlipProbability, maxRangeMeters, seed);
    }

    /** Returns a copy with the multi-tag heading standard deviation at 1 m, in degrees. */
    public VisionSimNoise headingStdDegAt1m(double degrees) {
        return new VisionSimNoise(fps, latencyMs, jitterMs, translationStdAt1m, degrees,
                singleTagMultiplier, dropProbability, ambiguityFlipProbability, maxRangeMeters, seed);
    }

    /** Returns a copy whose single-tag frames are this many times noisier than multi-tag frames. */
    public VisionSimNoise singleTagMultiplier(double multiplier) {
        return new VisionSimNoise(fps, latencyMs, jitterMs, translationStdAt1m, headingStdDegAt1m,
                multiplier, dropProbability, ambiguityFlipProbability, maxRangeMeters, seed);
    }

    /** Returns a copy with the probability that a frame is lost. */
    public VisionSimNoise dropProbability(double probability) {
        return new VisionSimNoise(fps, latencyMs, jitterMs, translationStdAt1m, headingStdDegAt1m,
                singleTagMultiplier, probability, ambiguityFlipProbability, maxRangeMeters, seed);
    }

    /** Returns a copy with the probability that a single-tag frame reports a mirrored pose. */
    public VisionSimNoise ambiguityFlipProbability(double probability) {
        return new VisionSimNoise(fps, latencyMs, jitterMs, translationStdAt1m, headingStdDegAt1m,
                singleTagMultiplier, dropProbability, probability, maxRangeMeters, seed);
    }

    /** Returns a copy with the maximum visible tag distance in meters. */
    public VisionSimNoise maxRangeMeters(double meters) {
        return new VisionSimNoise(fps, latencyMs, jitterMs, translationStdAt1m, headingStdDegAt1m,
                singleTagMultiplier, dropProbability, ambiguityFlipProbability, meters, seed);
    }

    /** Returns a copy with a seed for the deterministic random stream. */
    public VisionSimNoise seed(long randomSeed) {
        return new VisionSimNoise(fps, latencyMs, jitterMs, translationStdAt1m, headingStdDegAt1m,
                singleTagMultiplier, dropProbability, ambiguityFlipProbability, maxRangeMeters, randomSeed);
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
