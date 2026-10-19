package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.sysid.SysIdLog;
import ca.frc6390.athena.mechanism.sysid.SysIdSample;
import ca.frc6390.athena.mechanism.sysid.SysIdState;
import ca.frc6390.athena.mechanism.sysid.SysIdUnit;
import java.util.Objects;

/**
 * Engine behind {@link ca.frc6390.athena.mechanism.model.Characterization}: the four SysId test phases for one
 * control binding, run through the normal control output, constraints, ownership and arbitration.
 *
 * <p>Robot code does not use this class. It is public only because the model package builds on it; declare a
 * {@code Characterization} field instead.</p>
 */
public final class CharacterizationRoutine {
    private final ControlBinding<?> control;
    private final SysIdUnit unit;
    private final double rampRateVoltsPerSecond;
    private final double maxVoltage;
    private final double timeoutSeconds;
    private final double minPosition;
    private final double maxPosition;
    private final SysIdLog log;
    private boolean active;
    private boolean limitReached;

    private CharacterizationRoutine(
            ControlBinding<?> control,
            SysIdUnit unit,
            double rampRateVoltsPerSecond,
            double maxVoltage,
            double timeoutSeconds,
            double minPosition,
            double maxPosition,
            SysIdLog log) {
        this.control = Objects.requireNonNull(control, "control");
        this.unit = Objects.requireNonNull(unit, "unit");
        this.rampRateVoltsPerSecond = requirePositive(rampRateVoltsPerSecond, "ramp rate");
        this.maxVoltage = requirePositive(maxVoltage, "max voltage");
        this.timeoutSeconds = requirePositive(timeoutSeconds, "timeout");
        this.minPosition = minPosition;
        this.maxPosition = maxPosition;
        this.log = Objects.requireNonNull(log, "log");
    }

    /**
     * Creates the routine with defaults: 0.5 V/s ramp, 4 V limit, 10 s timeout, no range, and feedback units taken
     * from the feedback encoder.
     *
     * @param control binding with an output motor and explicit feedback
     * @param log receiver of every routine sample; the routine calls {@link SysIdLog#end()} when a test ends
     */
    public static CharacterizationRoutine forControl(ControlBinding<?> control, SysIdLog log) {
        ControlBinding<?> safeControl = Objects.requireNonNull(control, "control");
        if (safeControl.output() == null) {
            throw new IllegalStateException("Characterization requires a control output motor.");
        }
        if (safeControl.feedback() == null) {
            throw new IllegalStateException("Characterization requires explicit control feedback(...).");
        }
        return new CharacterizationRoutine(safeControl, inferUnit(safeControl), 0.5, 4.0, 10.0,
                Double.NaN, Double.NaN, log);
    }

    /** Copy with a different ramp rate in volts per second. */
    public CharacterizationRoutine withRampRate(double voltsPerSecond) {
        return copy(unit, voltsPerSecond, maxVoltage, timeoutSeconds, minPosition, maxPosition);
    }

    /** Copy with a different voltage limit. Ramps stop at it and steps use it. */
    public CharacterizationRoutine withMaxVoltage(double volts) {
        return copy(unit, rampRateVoltsPerSecond, volts, timeoutSeconds, minPosition, maxPosition);
    }

    /** Copy with a different per-test safety timeout in seconds. */
    public CharacterizationRoutine withTimeout(double seconds) {
        return copy(unit, rampRateVoltsPerSecond, maxVoltage, seconds, minPosition, maxPosition);
    }

    /**
     * Copy with a travel range, in the feedback's normalized units (rotations or meters). A test ends one cycle after
     * the position leaves the range. Pass NaN to leave a bound open.
     */
    public CharacterizationRoutine withRange(double minNormalized, double maxNormalized) {
        return copy(unit, rampRateVoltsPerSecond, maxVoltage, timeoutSeconds, minNormalized, maxNormalized);
    }

    /** The bound control binding. */
    public ControlBinding<?> control() {
        return control;
    }

    /** Feedback units used to normalize samples. */
    public SysIdUnit unit() {
        return unit;
    }

    /** Forward quasistatic ramp action. */
    public Action quasistaticForward() {
        return new Test(this, SysIdState.QUASISTATIC_FORWARD);
    }

    /** Reverse quasistatic ramp action. */
    public Action quasistaticReverse() {
        return new Test(this, SysIdState.QUASISTATIC_REVERSE);
    }

    /** Forward dynamic step action, held at the voltage limit. */
    public Action dynamicForward() {
        return new Test(this, SysIdState.DYNAMIC_FORWARD);
    }

    /** Reverse dynamic step action, held at the negated voltage limit. */
    public Action dynamicReverse() {
        return new Test(this, SysIdState.DYNAMIC_REVERSE);
    }

    boolean timedOut(double elapsedSeconds) {
        return limitReached || elapsedSeconds >= timeoutSeconds;
    }

    double voltage(SysIdState state, double elapsedSeconds) {
        double direction = switch (state) {
            case QUASISTATIC_FORWARD, DYNAMIC_FORWARD -> 1.0;
            case QUASISTATIC_REVERSE, DYNAMIC_REVERSE -> -1.0;
            case NONE -> 0.0;
        };
        double magnitude = switch (state) {
            case QUASISTATIC_FORWARD, QUASISTATIC_REVERSE ->
                    Math.min(maxVoltage, Math.max(0.0, elapsedSeconds) * rampRateVoltsPerSecond);
            case DYNAMIC_FORWARD, DYNAMIC_REVERSE -> maxVoltage;
            case NONE -> 0.0;
        };
        return direction * magnitude;
    }

    synchronized void record(ActionContext context, SysIdState state, double fallbackAppliedVoltage) {
        Objects.requireNonNull(context, "context");
        active = true;
        double voltage = readOrFallback(context.motor(control.output())::appliedVoltage, fallbackAppliedVoltage);
        double current = readOrFallback(context.motor(control.output())::statorCurrentAmps, Double.NaN);
        double position = unit.normalizePosition(control.feedback().position().position());
        double velocity = unit.normalizeVelocity(control.feedback().velocity().velocity());
        if ((!Double.isNaN(minPosition) && position < minPosition)
                || (!Double.isNaN(maxPosition) && position > maxPosition)) {
            limitReached = true;
        }
        log.record(new SysIdSample(state, unit.angular(), voltage, position, velocity, current));
    }

    synchronized void end() {
        if (active) {
            log.end();
        }
        active = false;
        limitReached = false;
    }

    private CharacterizationRoutine copy(SysIdUnit unit, double rampRate, double maxVoltage, double timeout,
            double min, double max) {
        return new CharacterizationRoutine(control, unit, rampRate, maxVoltage, timeout, min, max, log);
    }

    private static SysIdUnit inferUnit(ControlBinding<?> control) {
        if (control.feedback().position() instanceof EncoderDevice encoder) {
            return switch (encoder.units()) {
                case DEGREES -> SysIdUnit.DEGREES;
                case RADIANS -> SysIdUnit.RADIANS;
                case METERS -> SysIdUnit.METERS;
                case RAW, ROTATIONS -> SysIdUnit.ROTATIONS;
            };
        }
        return SysIdUnit.ROTATIONS;
    }

    private static double readOrFallback(DoubleRead read, double fallback) {
        try {
            double value = read.getAsDouble();
            return Double.isFinite(value) ? value : fallback;
        } catch (UnsupportedOperationException exception) {
            return fallback;
        }
    }

    private static double requirePositive(double value, String description) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException("Characterization " + description + " must be positive.");
        }
        return value;
    }

    @FunctionalInterface
    private interface DoubleRead {
        double getAsDouble();
    }

    /**
     * One test phase. Holding it runs the test; the runtime ends it on timeout, limit, or release.
     * Package-private: the runtime recognizes it, robot code only sees {@link Action}.
     */
    record Test(CharacterizationRoutine routine, SysIdState state) implements Action {
        Test {
            Objects.requireNonNull(routine, "routine");
            Objects.requireNonNull(state, "state");
            if (state == SysIdState.NONE) {
                throw new IllegalArgumentException("A characterization test requires an active test state.");
            }
        }

        Voltage output(double elapsedSeconds) {
            return new Voltage(routine, state, routine.voltage(state, elapsedSeconds));
        }
    }

    /** Voltage applied by one cycle of a {@link Test}. Package-private, like {@link Test}. */
    record Voltage(CharacterizationRoutine routine, SysIdState state, double volts)
            implements Action, Output.Voltage {
        Voltage {
            Objects.requireNonNull(routine, "routine");
            Objects.requireNonNull(state, "state");
            if (!Double.isFinite(volts)) {
                throw new IllegalArgumentException("Characterization voltage must be finite.");
            }
        }
    }
}
