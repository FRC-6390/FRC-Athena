package frc.robot.controls;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.signal.PositionSignal;
import ca.frc6390.athena.hardware.signal.VelocitySignal;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Objects;
import java.util.Set;

/** Example signal filters and fusion algorithms built on Athena's feedback interfaces. */
public final class FeedbackSignals {
    private static final double EPSILON = 1.0e-9;

    private FeedbackSignals() {
    }

    /** Describes one periodic encoder phase used by the modular decoder. */
    public record ModularInput(PositionSignal phase, double encoderTurnsPerMechanismTurn) {
        public ModularInput {
            Objects.requireNonNull(phase, "phase");
            if (!Double.isFinite(encoderTurnsPerMechanismTurn)
                    || encoderTurnsPerMechanismTurn == 0.0) {
                throw new IllegalArgumentException("Modular input ratio must be finite and non-zero.");
            }
        }
    }

    public static ModularInput input(PositionSignal phase, double encoderTurnsPerMechanismTurn) {
        return new ModularInput(phase, encoderTurnsPerMechanismTurn);
    }

    /** Reads a single-turn encoder's absolute phase in the interval [0, 1). */
    public static PositionSignal absolutePhase(EncoderDevice encoder) {
        return new AbsolutePhase(Objects.requireNonNull(encoder, "encoder"));
    }

    /**
     * Reconstructs absolute mechanism rotations from periodic encoder phases.
     *
     * <p>The range is half-open: its minimum is included and maximum excluded.
     * That matters because the combined encoder pattern repeats at the maximum.</p>
     */
    public static PositionSignal modular(
            Range range,
            double maxPhaseError,
            ModularInput... inputs) {
        return new ModularPosition(range, maxPhaseError, inputs);
    }

    /**
     * Tracks a fast relative encoder after initializing it from an absolute source.
     * Small absolute innovations correct drift; large jumps are rejected.
     */
    public static PositionSignal absoluteRelative(
            PositionSignal absolute,
            PositionSignal relative,
            double maxCorrection,
            double correctionGain) {
        return new AbsoluteRelativePosition(absolute, relative, maxCorrection, correctionGain);
    }

    /** Applies a one-pole low-pass filter to velocity. */
    public static VelocitySignal lowPass(VelocitySignal input, double alpha) {
        return new LowPassVelocity(input, alpha);
    }

    /** Uses the median finite position reported by a redundant sensor group. */
    public static PositionSignal median(PositionSignal... inputs) {
        return new MedianPosition(inputs);
    }

    private record AbsolutePhase(EncoderDevice encoder) implements PositionSignal {
        @Override
        public double position(ActionContext context) {
            double rotations = context.encoder(encoder).absolutePositionRotations();
            return wrap(encoder.positionFromRotations(rotations));
        }

        @Override
        public List<?> dependencies() {
            return List.of(encoder);
        }
    }

    private static final class ModularPosition implements PositionSignal {
        private final Range range;
        private final double maxPhaseError;
        private final List<ModularInput> inputs;
        private final List<?> dependencies;

        private ModularPosition(Range range, double maxPhaseError, ModularInput... inputs) {
            this.range = Objects.requireNonNull(range, "range");
            if (!Double.isFinite(range.minimum()) || !Double.isFinite(range.maximum())
                    || range.maximum() <= range.minimum()) {
                throw new IllegalArgumentException("Modular range must be finite and non-empty.");
            }
            if (!Double.isFinite(maxPhaseError) || maxPhaseError <= 0.0 || maxPhaseError >= 0.5) {
                throw new IllegalArgumentException("Maximum phase error must be between zero and half a turn.");
            }
            if (inputs == null || inputs.length < 2) {
                throw new IllegalArgumentException("Modular position requires at least two phase inputs.");
            }
            this.maxPhaseError = maxPhaseError;
            this.inputs = List.copyOf(Arrays.asList(inputs.clone()));
            dependencies = FeedbackSignals.dependencies(this.inputs.stream().map(ModularInput::phase).toList());
        }

        @Override
        public double position(ActionContext context) {
            double[] measured = new double[inputs.size()];
            for (int index = 0; index < inputs.size(); index++) {
                measured[index] = wrap(inputs.get(index).phase().position(context));
                if (!Double.isFinite(measured[index])) {
                    return Double.NaN;
                }
            }

            ModularInput reference = inputs.get(0);
            double referenceRatio = reference.encoderTurnsPerMechanismTurn();
            double scaledA = referenceRatio * range.minimum();
            double scaledB = referenceRatio * range.maximum();
            double lower = Math.min(scaledA, scaledB) - measured[0];
            double upper = Math.max(scaledA, scaledB) - measured[0];
            // Include one candidate beyond each boundary. Circular sensor noise
            // can move a phase across zero even when the mechanism remains just
            // inside the configured range.
            long firstIndex = (long) Math.ceil(lower - EPSILON) - 1L;
            long lastIndex = (long) Math.floor(upper + EPSILON) + 1L;
            if (lastIndex - firstIndex > 100_000L) {
                throw new IllegalStateException("Modular range produces too many reconstruction candidates.");
            }

            double bestPosition = Double.NaN;
            double bestScore = Double.POSITIVE_INFINITY;
            double secondScore = Double.POSITIVE_INFINITY;
            for (long candidateIndex = firstIndex; candidateIndex <= lastIndex; candidateIndex++) {
                double candidate = (measured[0] + candidateIndex) / referenceRatio;

                // Refine the discrete candidate using all phase residuals. This
                // preserves the CRT turn choice while avoiding a hard dependency
                // on the reference encoder's noise at its wrap boundary.
                double correctionNumerator = 0.0;
                double correctionDenominator = 0.0;
                for (int inputIndex = 0; inputIndex < inputs.size(); inputIndex++) {
                    double ratio = inputs.get(inputIndex).encoderTurnsPerMechanismTurn();
                    double expected = wrap(ratio * candidate);
                    double error = circularDifference(measured[inputIndex], expected);
                    correctionNumerator += ratio * error;
                    correctionDenominator += ratio * ratio;
                }
                candidate += correctionNumerator / correctionDenominator;
                if (candidate < range.minimum() - EPSILON || candidate >= range.maximum() - EPSILON) {
                    continue;
                }

                double score = 0.0;
                double largestError = 0.0;
                for (int inputIndex = 0; inputIndex < inputs.size(); inputIndex++) {
                    double expected = wrap(inputs.get(inputIndex).encoderTurnsPerMechanismTurn() * candidate);
                    double error = Math.abs(circularDifference(measured[inputIndex], expected));
                    largestError = Math.max(largestError, error);
                    score += error * error;
                }
                if (largestError > maxPhaseError) {
                    continue;
                }
                if (score < bestScore) {
                    secondScore = bestScore;
                    bestScore = score;
                    bestPosition = candidate;
                } else if (score < secondScore) {
                    secondScore = score;
                }
            }

            if (!Double.isFinite(bestPosition)
                    || secondScore - bestScore <= EPSILON) {
                return Double.NaN;
            }
            return bestPosition;
        }

        @Override
        public List<?> dependencies() {
            return dependencies;
        }
    }

    private static final class AbsoluteRelativePosition implements PositionSignal {
        private final PositionSignal absolute;
        private final PositionSignal relative;
        private final double maxCorrection;
        private final double correctionGain;
        private final List<?> dependencies;
        private boolean initialized;
        private double relativeOffset;
        private boolean hasLastSample;
        private double lastAbsolute;
        private double lastRelative;
        private double lastResult;

        private AbsoluteRelativePosition(
                PositionSignal absolute,
                PositionSignal relative,
                double maxCorrection,
                double correctionGain) {
            this.absolute = Objects.requireNonNull(absolute, "absolute");
            this.relative = Objects.requireNonNull(relative, "relative");
            if (!Double.isFinite(maxCorrection) || maxCorrection <= 0.0) {
                throw new IllegalArgumentException("Maximum correction must be positive.");
            }
            if (!Double.isFinite(correctionGain) || correctionGain < 0.0 || correctionGain > 1.0) {
                throw new IllegalArgumentException("Correction gain must be between zero and one.");
            }
            this.maxCorrection = maxCorrection;
            this.correctionGain = correctionGain;
            dependencies = FeedbackSignals.dependencies(List.of(absolute, relative));
        }

        @Override
        public double position(ActionContext context) {
            double relativePosition = relative.position(context);
            if (!Double.isFinite(relativePosition)) {
                return Double.NaN;
            }
            double absolutePosition = absolute.position(context);
            if (hasLastSample
                    && Double.compare(absolutePosition, lastAbsolute) == 0
                    && Double.compare(relativePosition, lastRelative) == 0) {
                return lastResult;
            }
            if (!initialized) {
                if (!Double.isFinite(absolutePosition)) {
                    return Double.NaN;
                }
                relativeOffset = absolutePosition - relativePosition;
                initialized = true;
            } else if (Double.isFinite(absolutePosition)) {
                double innovation = absolutePosition - (relativePosition + relativeOffset);
                if (Math.abs(innovation) <= maxCorrection) {
                    relativeOffset += innovation * correctionGain;
                }
            }
            lastAbsolute = absolutePosition;
            lastRelative = relativePosition;
            lastResult = relativePosition + relativeOffset;
            hasLastSample = true;
            return lastResult;
        }

        @Override
        public List<?> dependencies() {
            return dependencies;
        }
    }

    private static final class LowPassVelocity implements VelocitySignal {
        private final VelocitySignal input;
        private final double alpha;
        private boolean initialized;
        private double filtered;

        private LowPassVelocity(VelocitySignal input, double alpha) {
            this.input = Objects.requireNonNull(input, "input");
            if (!Double.isFinite(alpha) || alpha <= 0.0 || alpha > 1.0) {
                throw new IllegalArgumentException("Low-pass alpha must be greater than zero and at most one.");
            }
            this.alpha = alpha;
        }

        @Override
        public double velocity(ActionContext context) {
            double measurement = input.velocity(context);
            if (!Double.isFinite(measurement)) {
                return Double.NaN;
            }
            if (!initialized) {
                filtered = measurement;
                initialized = true;
            } else {
                filtered += alpha * (measurement - filtered);
            }
            return filtered;
        }

        @Override
        public List<?> dependencies() {
            return input.dependencies();
        }
    }

    private static final class MedianPosition implements PositionSignal {
        private final List<PositionSignal> inputs;
        private final List<?> dependencies;

        private MedianPosition(PositionSignal... inputs) {
            if (inputs == null || inputs.length < 3) {
                throw new IllegalArgumentException("Median fusion requires at least three position signals.");
            }
            this.inputs = List.copyOf(Arrays.asList(inputs.clone()));
            dependencies = FeedbackSignals.dependencies(this.inputs);
        }

        @Override
        public double position(ActionContext context) {
            double[] finite = new double[inputs.size()];
            int count = 0;
            for (PositionSignal input : inputs) {
                double value = input.position(context);
                if (Double.isFinite(value)) {
                    finite[count++] = value;
                }
            }
            if (count < inputs.size() / 2 + 1) {
                return Double.NaN;
            }
            Arrays.sort(finite, 0, count);
            int middle = count / 2;
            return count % 2 == 0
                    ? (finite[middle - 1] + finite[middle]) * 0.5
                    : finite[middle];
        }

        @Override
        public List<?> dependencies() {
            return dependencies;
        }
    }

    private static List<?> dependencies(List<? extends PositionSignal> inputs) {
        Set<Object> dependencies = new LinkedHashSet<>();
        for (PositionSignal input : inputs) {
            dependencies.addAll(input.dependencies());
        }
        return List.copyOf(dependencies);
    }

    private static double wrap(double rotations) {
        if (!Double.isFinite(rotations)) {
            return Double.NaN;
        }
        double wrapped = rotations - Math.floor(rotations);
        return wrapped >= 1.0 ? 0.0 : wrapped;
    }

    private static double circularDifference(double first, double second) {
        double difference = wrap(first - second);
        return difference > 0.5 ? difference - 1.0 : difference;
    }
}
