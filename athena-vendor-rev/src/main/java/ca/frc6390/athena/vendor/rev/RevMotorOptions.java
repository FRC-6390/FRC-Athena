package ca.frc6390.athena.vendor.rev;

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

    private int smartCurrentLimitAmps;
    private double openLoopRampSeconds;
    private boolean openLoopRampConfigured;
    private double closedLoopRampSeconds;
    private boolean closedLoopRampConfigured;
    private boolean resetSafeParameters;
    private boolean persistParameters = true;
    private int primaryEncoderCountsPerRevolution;
    private int primaryEncoderSignalPeriodMs;
    private int absoluteEncoderSignalPeriodMs;
    private int absoluteEncoderAverageDepth;
    private AbsoluteEncoderProfile absoluteEncoderProfile = AbsoluteEncoderProfile.GENERIC;
    private double voltageCompensationVolts;
    private boolean voltageCompensationConfigured;
    private int telemetrySignalPeriodMs;
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
     * @param amps limit in amps
     * @return this options object
     */
    public RevMotorOptions smartCurrentLimit(int amps) {
        smartCurrentLimitAmps = Math.max(0, amps);
        return this;
    }

    /**
     * Sets open-loop ramp time.
     *
     * @param seconds ramp seconds
     * @return this options object
     */
    public RevMotorOptions openLoopRampSeconds(double seconds) {
        openLoopRampSeconds = sanitize(seconds);
        openLoopRampConfigured = true;
        return this;
    }

    /**
     * Sets closed-loop ramp time.
     *
     * @param seconds ramp seconds
     * @return this options object
     */
    public RevMotorOptions closedLoopRampSeconds(double seconds) {
        closedLoopRampSeconds = sanitize(seconds);
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

    /** Sets the integrated/primary encoder status period; zero keeps the REV default. */
    public RevMotorOptions primaryEncoderSignalPeriodMs(int periodMs) {
        primaryEncoderSignalPeriodMs = Math.max(0, periodMs);
        return this;
    }

    /** Sets the attached absolute encoder status period; zero keeps the REV default. */
    public RevMotorOptions absoluteEncoderSignalPeriodMs(int periodMs) {
        absoluteEncoderSignalPeriodMs = Math.max(0, periodMs);
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
    public RevMotorOptions voltageCompensation(double volts) {
        voltageCompensationVolts = sanitize(volts);
        voltageCompensationConfigured = true;
        return this;
    }

    /** Sets applied-output, bus-voltage, and output-current status periods. */
    public RevMotorOptions telemetrySignalPeriodMs(int periodMs) {
        telemetrySignalPeriodMs = Math.max(0, periodMs);
        return this;
    }

    /** Enables the SPARK forward soft limit in raw primary-encoder rotations. */
    public RevMotorOptions forwardSoftLimitRotations(double rotations) {
        forwardSoftLimitRotations = finiteOrNaN(rotations);
        forwardSoftLimitConfigured = true;
        return this;
    }

    /** Enables the SPARK reverse soft limit in raw primary-encoder rotations. */
    public RevMotorOptions reverseSoftLimitRotations(double rotations) {
        reverseSoftLimitRotations = finiteOrNaN(rotations);
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
     * @return amps, or zero when unset
     */
    public int smartCurrentLimitAmps() {
        return smartCurrentLimitAmps;
    }

    /**
     * Returns open-loop ramp seconds.
     *
     * @return seconds
     */
    public double openLoopRampSeconds() {
        return openLoopRampSeconds;
    }

    boolean openLoopRampConfigured() { return openLoopRampConfigured; }

    /**
     * Returns closed-loop ramp seconds.
     *
     * @return seconds
     */
    public double closedLoopRampSeconds() {
        return closedLoopRampSeconds;
    }

    boolean closedLoopRampConfigured() { return closedLoopRampConfigured; }

    public boolean resetSafeParameters() { return resetSafeParameters; }

    public boolean persistParameters() { return persistParameters; }

    public int primaryEncoderCountsPerRevolution() { return primaryEncoderCountsPerRevolution; }

    public int primaryEncoderSignalPeriodMs() { return primaryEncoderSignalPeriodMs; }

    public int absoluteEncoderSignalPeriodMs() { return absoluteEncoderSignalPeriodMs; }

    public int absoluteEncoderAverageDepth() { return absoluteEncoderAverageDepth; }

    public AbsoluteEncoderProfile absoluteEncoderProfile() { return absoluteEncoderProfile; }

    public double voltageCompensationVolts() { return voltageCompensationVolts; }

    boolean voltageCompensationConfigured() { return voltageCompensationConfigured; }

    public int telemetrySignalPeriodMs() { return telemetrySignalPeriodMs; }

    public double forwardSoftLimitRotations() { return forwardSoftLimitRotations; }

    boolean forwardSoftLimitConfigured() { return forwardSoftLimitConfigured; }

    public double reverseSoftLimitRotations() { return reverseSoftLimitRotations; }

    boolean reverseSoftLimitConfigured() { return reverseSoftLimitConfigured; }

    public boolean forwardLimitSwitchEnabled() { return forwardLimitSwitchEnabled; }

    boolean forwardLimitSwitchConfigured() { return forwardLimitSwitchConfigured; }

    public boolean reverseLimitSwitchEnabled() { return reverseLimitSwitchEnabled; }

    boolean reverseLimitSwitchConfigured() { return reverseLimitSwitchConfigured; }

    public boolean forwardLimitSwitchNormallyClosed() { return forwardLimitSwitchNormallyClosed; }

    public boolean reverseLimitSwitchNormallyClosed() { return reverseLimitSwitchNormallyClosed; }

    private double sanitize(double seconds) {
        return Double.isFinite(seconds) ? Math.max(0.0, seconds) : 0.0;
    }

    private static double finiteOrNaN(double value) {
        return Double.isFinite(value) ? value : Double.NaN;
    }
}
