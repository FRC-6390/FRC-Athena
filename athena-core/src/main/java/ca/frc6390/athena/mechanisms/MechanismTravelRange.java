package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.sensors.limitswitch.GenericLimitSwitch;
import edu.wpi.first.math.MathUtil;
import java.util.Objects;
import java.util.WeakHashMap;

/**
 * Tracks the observed travel range for a mechanism and normalizes scalar values into the [0, 1]
 * interval. When limit switches specify explicit bounds, those are treated as fixed travel limits.
 * Otherwise the tracker learns extrema over time with sensible fallbacks for initial values.
 */
public final class MechanismTravelRange {

    private MechanismTravelRange() {
        /* utility */
    }

    private static final WeakHashMap<Mechanism, State> STATE_MAP = new WeakHashMap<>();

    public static double normalize(Mechanism mechanism, double value) {
        Objects.requireNonNull(mechanism);
        return stateFor(mechanism).normalize(mechanism, value);
    }

    public static void registerKnownRange(Mechanism mechanism, double min, double max) {
        if (!(max > min)) {
            return;
        }
        stateFor(mechanism).registerFixedRange(min, max);
    }

    private static State stateFor(Mechanism mechanism) {
        synchronized (STATE_MAP) {
            return STATE_MAP.computeIfAbsent(mechanism, key -> new State());
        }
    }

    private static final class State {
        private static final double DEFAULT_RANGE_GUESS = 1.0;
        private boolean seeded = false;
        private double min = Double.NaN;
        private double max = Double.NaN;
        private double fixedMin = Double.NaN;
        private double fixedMax = Double.NaN;
        private double observedMin = Double.POSITIVE_INFINITY;
        private double observedMax = Double.NEGATIVE_INFINITY;

        double normalize(Mechanism mechanism, double value) {
            seedIfNeeded(mechanism, value);
            if (!Double.isFinite(value)) {
                value = rangeMin();
            }
            observe(value);
            double lower = rangeMin();
            double upper = rangeMax();
            double span = upper - lower;
            if (!(span > 1e-6)) {
                return 0.0;
            }
            double clamped = MathUtil.clamp(value, lower, upper);
            return MathUtil.clamp((clamped - lower) / span, 0.0, 1.0);
        }

        private void seedIfNeeded(Mechanism mechanism, double initialValue) {
            if (seeded) {
                return;
            }
            seeded = true;

            double seededMin = Double.POSITIVE_INFINITY;
            double seededMax = Double.NEGATIVE_INFINITY;

            for (GenericLimitSwitch limit : mechanism.getLimitSwitches()) {
                double position = limit.getPosition();
                if (!Double.isFinite(position)) {
                    continue;
                }
                if (position < seededMin) {
                    seededMin = position;
                }
                if (position > seededMax) {
                    seededMax = position;
                }
                if (!Double.isFinite(fixedMin) || position < fixedMin) {
                    fixedMin = position;
                }
                if (!Double.isFinite(fixedMax) || position > fixedMax) {
                    fixedMax = position;
                }
            }

            if (hasFixedRange()) {
                min = fixedMin;
                max = fixedMax;
                observedMin = fixedMin;
                observedMax = fixedMax;
                return;
            }

            double position = mechanism.getPosition();
            double setpoint = mechanism.getSetpoint();
            double seedValue;
            if (Double.isFinite(position)) {
                seedValue = position;
            } else if (Double.isFinite(setpoint)) {
                seedValue = setpoint;
            } else if (Double.isFinite(initialValue)) {
                seedValue = initialValue;
            } else {
                seedValue = 0.0;
            }

            if (Double.isFinite(seededMin) && Double.isFinite(seededMax) && seededMax > seededMin) {
                min = seededMin;
                max = seededMax;
            } else {
                min = seedValue - (DEFAULT_RANGE_GUESS / 2.0);
                max = seedValue + (DEFAULT_RANGE_GUESS / 2.0);
            }
            observedMin = seedValue;
            observedMax = seedValue;
        }

        private void observe(double value) {
            if (!Double.isFinite(value)) {
                return;
            }
            if (value < observedMin) {
                observedMin = value;
            }
            if (value > observedMax) {
                observedMax = value;
            }
            if (hasFixedRange()) {
                return;
            }

            if (!Double.isFinite(min) || !Double.isFinite(max)) {
                min = observedMin;
                max = observedMax;
            }

            if (observedMax - observedMin > 1e-6) {
                min = observedMin;
                max = observedMax;
            }
        }

        private boolean hasFixedRange() {
            return Double.isFinite(fixedMin) && Double.isFinite(fixedMax) && fixedMax > fixedMin;
        }

        private double rangeMin() {
            if (hasFixedRange()) {
                return fixedMin;
            }
            if (Double.isFinite(min)) {
                return min;
            }
            if (Double.isFinite(observedMin)) {
                return observedMin;
            }
            return 0.0;
        }

        private double rangeMax() {
            if (hasFixedRange()) {
                return fixedMax;
            }
            if (Double.isFinite(max)) {
                return max;
            }
            if (Double.isFinite(observedMax)) {
                return observedMax;
            }
            return rangeMin() + DEFAULT_RANGE_GUESS;
        }

        private void registerFixedRange(double min, double max) {
            this.fixedMin = min;
            this.fixedMax = max;
            this.min = min;
            this.max = max;
            this.observedMin = min;
            this.observedMax = max;
            this.seeded = true;
        }
    }
}
