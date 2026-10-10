package ca.frc6390.athena.mechanism.model;

import ca.frc6390.athena.hardware.units.MotionKind;
import ca.frc6390.athena.mechanism.sysid.SysIdState;
import ca.frc6390.athena.hardware.units.UnitConversions;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.OptionalDouble;

/**
 * Closed-form estimators for each characterization parameter.
 *
 * <p>Conventions. Rotary positions are in rotations and the gravity angle is {@code 2*pi*position}.
 * Linear positions are in meters. Gravity is {@code kG * cos(theta + offset)} for rotary mechanisms
 * and a constant {@code kG} for linear ones.</p>
 *
 * <p>Feedforward equation, fitted by ordinary least squares on samples with {@code |v| > deadband}:</p>
 * <pre>
 *   V = kS_kinetic * sign(v) + kV * v + kA * a + kG * cos(theta + offset)      (rotary)
 *   V = kS_kinetic * sign(v) + kV * v + kA * a + kG                           (linear)
 * </pre>
 * <p>The gravity term is linear in {@code cos(theta)} and {@code sin(theta)}, so it is fitted as
 * {@code gc*cos(theta) + gs*sin(theta)} and converted to {@code kG = hypot(gc, gs)} and
 * {@code offset = atan2(-gs, gc)}.</p>
 *
 * <p>Static friction is the voltage at breakaway in a quasistatic ramp, minus the gravity term at that
 * position. Friction table bins hold the mean kinetic residual per position bin. Backlash is the
 * distance between the last moving sample before a reversal and the first moving sample after it.
 * Latency is the time from a dynamic step to the first sample above the velocity threshold.</p>
 */
public final class Identification {
    /** Velocity below which a sample counts as stationary for kinetic fits. */
    public static final double DEFAULT_VELOCITY_DEADBAND = 0.02;
    private static final double TWO_PI = 2.0 * Math.PI;

    /**
     * Feedforward fit result.
     *
     * @param kineticStaticVolts kinetic kS, the friction voltage while moving
     * @param kV velocity gain, volts per base-unit per second
     * @param kA acceleration gain, volts per base-unit per second squared
     * @param kG gravity amplitude in volts (linear: the constant)
     * @param gravityOffsetRadians phase offset of the gravity term, 0 for linear
     * @param standardErrors standard error per parameter: kineticStatic, kV, kA, kG
     * @param rSquared coefficient of determination of the kinetic fit
     * @param conditionNumber condition number of the regressors
     * @param samples rows used
     */
    public record FeedforwardFit(
            double kineticStaticVolts,
            double kV,
            double kA,
            double kG,
            double gravityOffsetRadians,
            Map<String, Double> standardErrors,
            double rSquared,
            double conditionNumber,
            int samples) {
        public FeedforwardFit {
            standardErrors = Map.copyOf(standardErrors);
        }
    }

    /**
     * Payload fit result.
     *
     * @param deltaKG change of the gravity amplitude relative to the baseline model
     * @param standardError standard error of the fitted amplitude
     * @param rSquared coefficient of determination
     * @param samples rows used
     */
    public record PayloadEstimate(double deltaKG, double standardError, double rSquared, int samples) {
    }

    private Identification() {
    }

    /**
     * Returns the regressor row for one sample. Rotary rows are
     * {@code [sign(v), v, a, cos(2 pi p), sin(2 pi p)]}, linear rows are {@code [sign(v), v, a, 1]}.
     * The sign is 0 inside the deadband.
     */
    public static double[] features(MotionKind kind, double position, double velocity,
            double acceleration, double deadband) {
        double sign = velocity > deadband ? 1.0 : velocity < -deadband ? -1.0 : 0.0;
        if (kind == MotionKind.ROTARY) {
            double angle = TWO_PI * position;
            return new double[] {sign, velocity, acceleration, Math.cos(angle), Math.sin(angle)};
        }
        return new double[] {sign, velocity, acceleration, 1.0};
    }

    /** Number of regressor columns used for the kind of motion. */
    public static int columns(MotionKind kind) {
        return kind == MotionKind.ROTARY ? 5 : 4;
    }

    /**
     * Returns the factor that converts a binding's unit to base units: rotations for angular units,
     * meters for distance units, and 1 for raw bindings.
     */
    public static double baseFactor(Unit unit) {
        if (unit instanceof AngleUnit) {
            return UnitConversions.factor(unit, Units.Rotations);
        }
        if (unit instanceof DistanceUnit) {
            return UnitConversions.factor(unit, Units.Meters);
        }
        return 1.0;
    }

    /** Fits kS (kinetic), kV, kA and gravity by OLS on samples moving faster than the deadband. */
    public static FeedforwardFit fitFeedforward(List<CharacterizationSample> samples, MotionKind kind,
            double deadband) {
        List<double[]> rows = new ArrayList<>();
        List<Double> targets = new ArrayList<>();
        for (CharacterizationSample sample : samples) {
            if (Math.abs(sample.velocity()) <= deadband) {
                continue;
            }
            rows.add(features(kind, sample.position(), sample.velocity(), sample.acceleration(), deadband));
            targets.add(sample.voltage());
        }
        LeastSquares.Fit fit = LeastSquares.fit(rows.toArray(new double[0][]), toArray(targets));
        double[] beta = fit.coefficients();
        double[] se = fit.standardErrors();
        double kG;
        double offset;
        double gravitySe;
        if (kind == MotionKind.ROTARY) {
            double gc = beta[3];
            double gs = beta[4];
            kG = Math.hypot(gc, gs);
            offset = Math.atan2(-gs, gc);
            gravitySe = Math.max(se[3], se[4]);
        } else {
            kG = beta[3];
            offset = 0.0;
            gravitySe = se[3];
        }
        Map<String, Double> errors = new LinkedHashMap<>();
        errors.put("kineticStatic", se[0]);
        errors.put("kV", se[1]);
        errors.put("kA", se[2]);
        errors.put("kG", gravitySe);
        return new FeedforwardFit(beta[0], beta[1], beta[2], kG, offset, errors,
                fit.rSquared(), fit.conditionNumber(), fit.samples());
    }

    /** Returns the gravity term of a fit at a position, in volts. */
    public static double gravityVolts(MotionKind kind, double kG, double offsetRadians, double position) {
        if (kind == MotionKind.ROTARY) {
            return kG * Math.cos(TWO_PI * position + offsetRadians);
        }
        return kG;
    }

    /**
     * Static friction from breakaway: the first quasistatic sample above {@code velocityThreshold} in
     * each ramp gives the voltage needed to start moving. Gravity is subtracted at that position.
     */
    public static OptionalDouble staticFriction(List<CharacterizationSample> samples, MotionKind kind,
            FeedforwardFit fit, double velocityThreshold) {
        double sum = 0.0;
        int events = 0;
        SysIdState runState = null;
        boolean armed = false;
        for (CharacterizationSample sample : samples) {
            if (!isQuasistatic(sample.state())) {
                runState = null;
                continue;
            }
            if (sample.state() != runState) {
                runState = sample.state();
                // Breakaway is only meaningful when the ramp starts from rest.
                armed = Math.abs(sample.velocity()) <= velocityThreshold;
            }
            if (armed && Math.abs(sample.velocity()) > velocityThreshold) {
                double gravity = gravityVolts(kind, fit.kG(), fit.gravityOffsetRadians(), sample.position());
                sum += Math.signum(sample.velocity()) * (sample.voltage() - gravity);
                events++;
                armed = false;
            }
        }
        return events == 0 ? OptionalDouble.empty() : OptionalDouble.of(sum / events);
    }

    /**
     * Kinetic friction offset versus position. Each bin holds the mean of
     * {@code sign(v) * (V - kV v - kA a - gravity) - kS}. Empty input gives an empty table.
     */
    public static FrictionTable frictionTable(List<CharacterizationSample> samples, MotionKind kind,
            FeedforwardFit fit, double deadband, int bins) {
        if (bins <= 0) {
            throw new IllegalArgumentException("Friction table needs at least one bin.");
        }
        List<double[]> points = new ArrayList<>();
        for (CharacterizationSample sample : samples) {
            if (Math.abs(sample.velocity()) <= deadband) {
                continue;
            }
            double gravity = gravityVolts(kind, fit.kG(), fit.gravityOffsetRadians(), sample.position());
            double residual = Math.signum(sample.velocity())
                    * (sample.voltage() - fit.kV() * sample.velocity() - fit.kA() * sample.acceleration() - gravity)
                    - fit.kineticStaticVolts();
            points.add(new double[] {sample.position(), residual});
        }
        if (points.isEmpty()) {
            return FrictionTable.EMPTY;
        }
        double min = Double.POSITIVE_INFINITY;
        double max = Double.NEGATIVE_INFINITY;
        for (double[] point : points) {
            min = Math.min(min, point[0]);
            max = Math.max(max, point[0]);
        }
        if (!(max - min > 1.0e-9)) {
            double mean = 0.0;
            for (double[] point : points) {
                mean += point[1];
            }
            return new FrictionTable(List.of(min), List.of(mean / points.size()));
        }
        double width = (max - min) / bins;
        double[] sums = new double[bins];
        int[] counts = new int[bins];
        for (double[] point : points) {
            int bin = Math.min(bins - 1, (int) ((point[0] - min) / width));
            sums[bin] += point[1];
            counts[bin]++;
        }
        List<Double> positions = new ArrayList<>();
        List<Double> volts = new ArrayList<>();
        for (int bin = 0; bin < bins; bin++) {
            if (counts[bin] == 0) {
                continue;
            }
            positions.add(min + (bin + 0.5) * width);
            volts.add(sums[bin] / counts[bin]);
        }
        return new FrictionTable(positions, volts);
    }

    /**
     * Backlash from forward-then-reverse ramps. For each reversal, the distance between the last
     * moving forward sample and the first moving reverse sample is one lost-motion measurement.
     */
    public static OptionalDouble backlash(List<CharacterizationSample> samples, double deadband) {
        double sum = 0.0;
        int events = 0;
        Double lastForwardPosition = null;
        boolean forwardSeen = false;
        for (CharacterizationSample sample : samples) {
            if (sample.state() == SysIdState.QUASISTATIC_FORWARD && sample.velocity() > deadband) {
                lastForwardPosition = sample.position();
                forwardSeen = true;
            } else if (sample.state() == SysIdState.QUASISTATIC_REVERSE && forwardSeen
                    && sample.velocity() < -deadband) {
                sum += Math.abs(sample.position() - lastForwardPosition);
                events++;
                forwardSeen = false;
            }
        }
        return events == 0 ? OptionalDouble.empty() : OptionalDouble.of(sum / events);
    }

    /**
     * Command-to-motion latency from dynamic steps. Latency is the time from the first sample of a
     * step run to the first sample whose speed reaches {@code velocityThreshold}.
     */
    public static OptionalDouble latency(List<CharacterizationSample> samples, double velocityThreshold) {
        double sum = 0.0;
        int events = 0;
        SysIdState runState = null;
        double runStart = Double.NaN;
        boolean armed = false;
        for (CharacterizationSample sample : samples) {
            if (sample.state() != SysIdState.DYNAMIC_FORWARD && sample.state() != SysIdState.DYNAMIC_REVERSE) {
                runState = null;
                continue;
            }
            if (sample.state() != runState) {
                runState = sample.state();
                runStart = sample.timeSeconds();
                armed = true;
            }
            if (armed && Math.abs(sample.velocity()) >= velocityThreshold) {
                sum += Math.max(0.0, sample.timeSeconds() - runStart);
                events++;
                armed = false;
            }
        }
        return events == 0 ? OptionalDouble.empty() : OptionalDouble.of(sum / events);
    }

    /**
     * Payload delta from holding samples. Each row is
     * {@code V - sign(state) * staticVolts - kV v - kA a = kG' * g(theta)}, where {@code g} is
     * {@code cos(2 pi p + offset)} for rotary and 1 for linear. The coefficient is the total loaded
     * amplitude, and the delta is its difference from the baseline {@code kG}.
     *
     * @throws IllegalArgumentException when there are too few holding samples
     */
    public static PayloadEstimate payload(List<CharacterizationSample> holds, MotionKind kind,
            FeedforwardFit baseline, double staticVolts) {
        double[][] rows = new double[holds.size()][1];
        double[] targets = new double[holds.size()];
        for (int i = 0; i < holds.size(); i++) {
            CharacterizationSample hold = holds.get(i);
            double direction = hold.state() == SysIdState.QUASISTATIC_REVERSE ? -1.0 : 1.0;
            targets[i] = hold.voltage() - direction * staticVolts
                    - baseline.kV() * hold.velocity() - baseline.kA() * hold.acceleration();
            rows[i][0] = kind == MotionKind.ROTARY
                    ? Math.cos(TWO_PI * hold.position() + baseline.gravityOffsetRadians())
                    : 1.0;
        }
        LeastSquares.Fit fit = LeastSquares.fit(rows, targets);
        return new PayloadEstimate(fit.coefficients()[0] - baseline.kG(), fit.standardErrors()[0],
                fit.rSquared(), fit.samples());
    }

    private static boolean isQuasistatic(SysIdState state) {
        return state == SysIdState.QUASISTATIC_FORWARD || state == SysIdState.QUASISTATIC_REVERSE;
    }

    private static double[] toArray(List<Double> values) {
        double[] array = new double[values.size()];
        for (int i = 0; i < array.length; i++) {
            array[i] = values.get(i);
        }
        return array;
    }
}
