package ca.frc6390.athena.mechanism.sysid;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.ControlBinding;
import java.util.Iterator;
import java.util.Objects;
import java.util.ServiceConfigurationError;
import java.util.ServiceLoader;

/**
 * SysId characterization actions for one control binding.
 *
 * <p>Run each action with a held binding. Athena applies the voltage through the
 * normal control output, constraints, ownership, and arbitration paths.</p>
 */
public final class ControlSysId {
    private static final double DEFAULT_RAMP_RATE = 1.0;
    private static final double DEFAULT_STEP_VOLTAGE = 7.0;
    private static final double DEFAULT_TIMEOUT = 10.0;

    private final ControlBinding control;
    private final String name;
    private final SysIdUnit unit;
    private final double rampRateVoltsPerSecond;
    private final double stepVoltage;
    private final double timeoutSeconds;
    private final SysIdLog configuredLog;
    private SysIdLog activeLog;
    private boolean active;

    private ControlSysId(
            ControlBinding control,
            String name,
            SysIdUnit unit,
            double rampRateVoltsPerSecond,
            double stepVoltage,
            double timeoutSeconds,
            SysIdLog configuredLog) {
        this.control = Objects.requireNonNull(control, "control");
        this.name = requireName(name);
        this.unit = Objects.requireNonNull(unit, "unit");
        this.rampRateVoltsPerSecond = requirePositive(rampRateVoltsPerSecond, "SysId ramp rate");
        this.stepVoltage = requirePositive(stepVoltage, "SysId step voltage");
        this.timeoutSeconds = requirePositive(timeoutSeconds, "SysId timeout");
        this.configuredLog = configuredLog;
    }

    public static ControlSysId forControl(ControlBinding control) {
        ControlBinding safeControl = Objects.requireNonNull(control, "control");
        if (safeControl.output() == null) {
            throw new IllegalStateException("SysId requires a control output motor.");
        }
        if (safeControl.feedback() == null) {
            throw new IllegalStateException("SysId requires explicit control feedback(...).");
        }
        return new ControlSysId(
                safeControl,
                safeControl.output().defaultName(),
                inferUnit(safeControl),
                DEFAULT_RAMP_RATE,
                DEFAULT_STEP_VOLTAGE,
                DEFAULT_TIMEOUT,
                null);
    }

    /** Sets the WPILog routine name. The output motor name is used by default. */
    public ControlSysId name(String name) {
        return copy(name, unit, rampRateVoltsPerSecond, stepVoltage, timeoutSeconds, configuredLog);
    }

    /** Declares the units returned by this binding's feedback signals. */
    /**
     * Declares the units returned by custom feedback signals.
     *
     * <p>Encoder feedback infers this from {@code EncoderDevice.units()}. RAW
     * encoder feedback is treated as rotations.</p>
     */
    public ControlSysId feedbackUnit(SysIdUnit unit) {
        return copy(name, Objects.requireNonNull(unit, "unit"), rampRateVoltsPerSecond,
                stepVoltage, timeoutSeconds, configuredLog);
    }

    /** Sets the positive quasistatic ramp magnitude in volts per second. */
    public ControlSysId rampRate(double voltsPerSecond) {
        return copy(name, unit, voltsPerSecond, stepVoltage, timeoutSeconds, configuredLog);
    }

    /** Sets the positive dynamic-test step magnitude in volts. */
    public ControlSysId stepVoltage(double volts) {
        return copy(name, unit, rampRateVoltsPerSecond, volts, timeoutSeconds, configuredLog);
    }

    /** Sets the safety timeout shared by all four tests. */
    public ControlSysId timeout(double seconds) {
        return copy(name, unit, rampRateVoltsPerSecond, stepVoltage, seconds, configuredLog);
    }

    /** Supplies an explicit logger, primarily for custom logging systems and tests. */
    public ControlSysId logger(SysIdLog log) {
        return copy(name, unit, rampRateVoltsPerSecond, stepVoltage, timeoutSeconds,
                Objects.requireNonNull(log, "log"));
    }

    /** Returns the forward voltage-ramp action. */
    public Action quasistaticForward() {
        return Actions.sysId(this, SysIdState.QUASISTATIC_FORWARD);
    }

    /** Returns the reverse voltage-ramp action. */
    public Action quasistaticReverse() {
        return Actions.sysId(this, SysIdState.QUASISTATIC_REVERSE);
    }

    /** Returns the forward step-voltage action. */
    public Action dynamicForward() {
        return Actions.sysId(this, SysIdState.DYNAMIC_FORWARD);
    }

    /** Returns the reverse step-voltage action. */
    public Action dynamicReverse() {
        return Actions.sysId(this, SysIdState.DYNAMIC_REVERSE);
    }

    /** Returns the characterized control binding. */
    public ControlBinding control() {
        return control;
    }

    public boolean timedOut(double elapsedSeconds) {
        return elapsedSeconds >= timeoutSeconds;
    }

    public double voltage(SysIdState state, double elapsedSeconds) {
        double direction = switch (state) {
            case QUASISTATIC_FORWARD, DYNAMIC_FORWARD -> 1.0;
            case QUASISTATIC_REVERSE, DYNAMIC_REVERSE -> -1.0;
            case NONE -> 0.0;
        };
        double magnitude = switch (state) {
            case QUASISTATIC_FORWARD, QUASISTATIC_REVERSE ->
                    Math.min(12.0, Math.max(0.0, elapsedSeconds) * rampRateVoltsPerSecond);
            case DYNAMIC_FORWARD, DYNAMIC_REVERSE -> stepVoltage;
            case NONE -> 0.0;
        };
        return direction * magnitude;
    }

    public synchronized void record(
            ActionContext context,
            SysIdState state,
            double fallbackAppliedVoltage) {
        Objects.requireNonNull(context, "context");
        if (!active) {
            activeLog = configuredLog == null ? discoverLog() : configuredLog;
            active = true;
        }
        double voltage = readOrFallback(
                context.motor(control.output())::appliedVoltage,
                fallbackAppliedVoltage);
        double current = readOrFallback(
                context.motor(control.output())::statorCurrentAmps,
                Double.NaN);
        double position = control.feedback().position().position();
        double velocity = control.feedback().velocity().velocity();
        activeLog.record(new SysIdSample(
                state,
                unit.angular(),
                voltage,
                unit.normalizePosition(position),
                unit.normalizeVelocity(velocity),
                current));
    }

    public synchronized void end() {
        if (active && activeLog != null) {
            activeLog.end();
        }
        active = false;
        activeLog = null;
    }

    private SysIdLog discoverLog() {
        Iterator<SysIdLogProvider> providers = ServiceLoader.load(SysIdLogProvider.class).iterator();
        while (true) {
            try {
                if (!providers.hasNext()) {
                    break;
                }
                SysIdLogProvider provider = providers.next();
                SysIdLog log = provider.open(name, control.output().defaultName());
                if (log != null) {
                    return log;
                }
            } catch (ServiceConfigurationError error) {
                throw new IllegalStateException("Unable to load the Athena SysId log provider.", error);
            }
        }
        throw new IllegalStateException(
                "No SysId log provider is installed. Add the athena-wpilib module or configure logger(...).");
    }

    private ControlSysId copy(
            String name,
            SysIdUnit unit,
            double rampRate,
            double stepVoltage,
            double timeout,
            SysIdLog log) {
        return new ControlSysId(control, name, unit, rampRate, stepVoltage, timeout, log);
    }

    private static SysIdUnit inferUnit(ControlBinding control) {
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

    private static String requireName(String name) {
        if (name == null || name.isBlank()) {
            throw new IllegalArgumentException("SysId name must not be blank.");
        }
        return name;
    }

    private static double requirePositive(double value, String description) {
        if (!Double.isFinite(value) || value <= 0.0) {
            throw new IllegalArgumentException(description + " must be positive.");
        }
        return value;
    }

    @FunctionalInterface
    private interface DoubleRead {
        double getAsDouble();
    }
}
