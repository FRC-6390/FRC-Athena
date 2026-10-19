package ca.frc6390.athena.vendor.rev;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;

/**
 * REV-specific motor options stored in Athena motor declarations.
 */
public final class RevMotorOptions {
    /** Supported duty-cycle absolute encoder calibrations. */
    public enum AbsoluteEncoderProfile {
        GENERIC,
        REV_THROUGH_BORE_V1,
        REV_THROUGH_BORE_V2
    }

    private double smartCurrentLimitAmps;
    private boolean preserveDeviceCurrentLimit;
    private double openLoopRampSeconds;
    private boolean openLoopRampConfigured;
    private double closedLoopRampSeconds;
    private boolean closedLoopRampConfigured;
    private boolean resetSafeParameters;
    private boolean persistParameters = true;
    private int primaryEncoderCountsPerRevolution;
    private double primaryEncoderSignalPeriodMs;
    private double absoluteEncoderSignalPeriodMs;
    private int absoluteEncoderAverageDepth;
    private AbsoluteEncoderProfile absoluteEncoderProfile = AbsoluteEncoderProfile.GENERIC;
    private double voltageCompensationVolts;
    private boolean voltageCompensationConfigured;
    private double telemetrySignalPeriodMs;
    private double forwardSoftLimitRotations = Double.NaN;
    private boolean forwardSoftLimitConfigured;
    private double reverseSoftLimitRotations = Double.NaN;
    private boolean reverseSoftLimitConfigured;
    private boolean forwardLimitSwitchEnabled;
    private boolean forwardLimitSwitchConfigured;
    private boolean reverseLimitSwitchEnabled;
    private boolean reverseLimitSwitchConfigured;
    private boolean forwardLimitSwitchNormallyClosed;
    private boolean reverseLimitSwitchNormallyClosed;

    /**
     * Sets smart current limit.
     *
     * @param limit limit; negative values are treated as zero
     * @return this options object
     */
    public RevMotorOptions smartCurrentLimit(Current limit) {
        smartCurrentLimitAmps = sanitize(limit == null ? Double.NaN : limit.in(Amps));
        return this;
    }

    /**
     * Keeps the smart current limit stored on the SPARK (for example from the REV Hardware Client)
     * when robot code declares no limit. By default Athena applies a conservative per-motor default
     * (NEO 40 A, NEO 550 20 A, NEO Vortex 60 A) instead.
     *
     * @param preserve true to leave the device's stored limit untouched
     * @return this options object
     */
    public RevMotorOptions preserveDeviceCurrentLimit(boolean preserve) {
        preserveDeviceCurrentLimit = preserve;
        return this;
    }

    /**
     * Sets open-loop ramp time.
     *
     * @param ramp time to ramp to full output; non-finite or negative values are treated as zero
     * @return this options object
     */
    public RevMotorOptions openLoopRamp(Time ramp) {
        openLoopRampSeconds = sanitize(ramp == null ? Double.NaN : ramp.in(Seconds));
        openLoopRampConfigured = true;
        return this;
    }

    /**
     * Sets closed-loop ramp time.
     *
     * @param ramp time to ramp to full output; non-finite or negative values are treated as zero
     * @return this options object
     */
    public RevMotorOptions closedLoopRamp(Time ramp) {
        closedLoopRampSeconds = sanitize(ramp == null ? Double.NaN : ramp.in(Seconds));
        closedLoopRampConfigured = true;
        return this;
    }

    /**
     * Controls whether initial activation resets safe parameters before applying Athena's partial
     * configuration. Defaults to false so settings owned by the REV Hardware Client are preserved.
     */
    public RevMotorOptions resetSafeParameters(boolean reset) {
        resetSafeParameters = reset;
        return this;
    }

    /** Controls whether initial configuration survives controller power cycles. Defaults to true. */
    public RevMotorOptions persistParameters(boolean persist) {
        persistParameters = persist;
        return this;
    }

    /** Configures the quadrature CPR used as a brushed motor's primary encoder. */
    public RevMotorOptions primaryEncoderCountsPerRevolution(int counts) {
        primaryEncoderCountsPerRevolution = Math.max(0, counts);
        return this;
    }

    /**
     * Sets the integrated/primary encoder status period; zero keeps the REV default. The SPARK takes whole
     * milliseconds, so the period is rounded to the nearest millisecond when applied.
     */
    public RevMotorOptions primaryEncoderSignalPeriod(Time period) {
        primaryEncoderSignalPeriodMs = sanitize(period == null ? Double.NaN : period.in(Milliseconds));
        return this;
    }

    /**
     * Sets the attached absolute encoder status period; zero keeps the REV default. The SPARK takes whole
     * milliseconds, so the period is rounded to the nearest millisecond when applied.
     */
    public RevMotorOptions absoluteEncoderSignalPeriod(Time period) {
        absoluteEncoderSignalPeriodMs = sanitize(period == null ? Double.NaN : period.in(Milliseconds));
        return this;
    }

    /** Sets the attached absolute encoder averaging depth; zero keeps the REV default. */
    public RevMotorOptions absoluteEncoderAverageDepth(int depth) {
        if (depth != 0 && (depth < 1 || depth > 128 || (depth & (depth - 1)) != 0)) {
            throw new IllegalArgumentException("REV absolute encoder average depth must be 0 or a power of two from 1 to 128.");
        }
        absoluteEncoderAverageDepth = depth;
        return this;
    }

    /** Selects pulse calibration for a controller-attached duty-cycle absolute encoder. */
    public RevMotorOptions absoluteEncoderProfile(AbsoluteEncoderProfile profile) {
        absoluteEncoderProfile = profile == null ? AbsoluteEncoderProfile.GENERIC : profile;
        return this;
    }

    /** Enables voltage compensation at the requested nominal voltage; zero disables it. */
    public RevMotorOptions voltageCompensation(Voltage nominal) {
        voltageCompensationVolts = sanitize(nominal == null ? Double.NaN : nominal.in(Volts));
        voltageCompensationConfigured = true;
        return this;
    }

    /**
     * Sets applied-output, bus-voltage, and output-current status periods. The SPARK takes whole milliseconds, so
     * the period is rounded to the nearest millisecond when applied.
     */
    public RevMotorOptions telemetrySignalPeriod(Time period) {
        telemetrySignalPeriodMs = sanitize(period == null ? Double.NaN : period.in(Milliseconds));
        return this;
    }

    /** Enables the SPARK forward soft limit in primary-encoder rotations. */
    public RevMotorOptions forwardSoftLimit(Angle limit) {
        forwardSoftLimitRotations = finiteOrNaN(limit == null ? Double.NaN : limit.in(Rotations));
        forwardSoftLimitConfigured = true;
        return this;
    }

    /** Enables the SPARK reverse soft limit in primary-encoder rotations. */
    public RevMotorOptions reverseSoftLimit(Angle limit) {
        reverseSoftLimitRotations = finiteOrNaN(limit == null ? Double.NaN : limit.in(Rotations));
        reverseSoftLimitConfigured = true;
        return this;
    }

    /** Enables/disables the forward data-port limit switch and selects its polarity. */
    public RevMotorOptions forwardLimitSwitch(boolean enabled, boolean normallyClosed) {
        forwardLimitSwitchEnabled = enabled;
        forwardLimitSwitchConfigured = true;
        forwardLimitSwitchNormallyClosed = normallyClosed;
        return this;
    }

    /** Enables/disables the reverse data-port limit switch and selects its polarity. */
    public RevMotorOptions reverseLimitSwitch(boolean enabled, boolean normallyClosed) {
        reverseLimitSwitchEnabled = enabled;
        reverseLimitSwitchConfigured = true;
        reverseLimitSwitchNormallyClosed = normallyClosed;
        return this;
    }

    /**
     * Returns smart current limit.
     *
     * @return limit, or zero when unset
     */
    public Current smartCurrentLimit() {
        return Amps.of(smartCurrentLimitAmps);
    }

    /**
     * Returns open-loop ramp time.
     *
     * @return ramp time
     */
    public Time openLoopRamp() {
        return Seconds.of(openLoopRampSeconds);
    }

    boolean openLoopRampConfigured() { return openLoopRampConfigured; }

    /**
     * Returns closed-loop ramp time.
     *
     * @return ramp time
     */
    public Time closedLoopRamp() {
        return Seconds.of(closedLoopRampSeconds);
    }

    boolean closedLoopRampConfigured() { return closedLoopRampConfigured; }

    public boolean preserveDeviceCurrentLimit() { return preserveDeviceCurrentLimit; }

    public boolean resetSafeParameters() { return resetSafeParameters; }

    public boolean persistParameters() { return persistParameters; }

    public int primaryEncoderCountsPerRevolution() { return primaryEncoderCountsPerRevolution; }

    public Time primaryEncoderSignalPeriod() { return Milliseconds.of(primaryEncoderSignalPeriodMs); }

    public Time absoluteEncoderSignalPeriod() { return Milliseconds.of(absoluteEncoderSignalPeriodMs); }

    public int absoluteEncoderAverageDepth() { return absoluteEncoderAverageDepth; }

    public AbsoluteEncoderProfile absoluteEncoderProfile() { return absoluteEncoderProfile; }

    public Voltage voltageCompensation() { return Volts.of(voltageCompensationVolts); }

    boolean voltageCompensationConfigured() { return voltageCompensationConfigured; }

    public Time telemetrySignalPeriod() { return Milliseconds.of(telemetrySignalPeriodMs); }

    public Angle forwardSoftLimit() { return Rotations.of(forwardSoftLimitRotations); }

    boolean forwardSoftLimitConfigured() { return forwardSoftLimitConfigured; }

    public Angle reverseSoftLimit() { return Rotations.of(reverseSoftLimitRotations); }

    boolean reverseSoftLimitConfigured() { return reverseSoftLimitConfigured; }

    public boolean forwardLimitSwitchEnabled() { return forwardLimitSwitchEnabled; }

    boolean forwardLimitSwitchConfigured() { return forwardLimitSwitchConfigured; }

    public boolean reverseLimitSwitchEnabled() { return reverseLimitSwitchEnabled; }

    boolean reverseLimitSwitchConfigured() { return reverseLimitSwitchConfigured; }

    public boolean forwardLimitSwitchNormallyClosed() { return forwardLimitSwitchNormallyClosed; }

    public boolean reverseLimitSwitchNormallyClosed() { return reverseLimitSwitchNormallyClosed; }

    private static double sanitize(double value) {
        return Double.isFinite(value) ? Math.max(0.0, value) : 0.0;
    }


    private static double finiteOrNaN(double value) {
        return Double.isFinite(value) ? value : Double.NaN;
    }
}
