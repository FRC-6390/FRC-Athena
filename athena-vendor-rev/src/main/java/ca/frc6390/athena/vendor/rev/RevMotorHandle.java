package ca.frc6390.athena.vendor.rev;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Time;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.api.hardware.MotorControllerKinds;
import ca.frc6390.athena.api.hardware.MotorTechnology;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.ConfigurationQueue;
import ca.frc6390.athena.hardware.backend.ConfigurationStatus;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.backend.OpenLoopCommandFilter;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.LimitSwitchConfig.Behavior;
import com.revrobotics.spark.config.LimitSwitchConfig.Type;
import java.util.Objects;

/**
 * REV Spark motor handle backed by REVLib.
 */
public final class RevMotorHandle implements MotorHandle, AutoCloseable {
    /** Smart current limit applied to a NEO when robot code declares no limit. */
    static final int DEFAULT_NEO_CURRENT_LIMIT_AMPS = 40;
    /** Smart current limit applied to a NEO 550 when robot code declares no limit. */
    static final int DEFAULT_NEO_550_CURRENT_LIMIT_AMPS = 20;
    /** Smart current limit applied to a NEO Vortex when robot code declares no limit. */
    static final int DEFAULT_NEO_VORTEX_CURRENT_LIMIT_AMPS = 60;

    private final MotorDevice device;
    private final RevMotorOptions options;
    private final SparkController controller;
    private final ConfigurationQueue configuration;
    private boolean activated;
    private boolean inputsFresh;
    private final OpenLoopCommandFilter openLoop = new OpenLoopCommandFilter();
    /** Refreshes since the motor temperature was last read; see {@link #TEMPERATURE_REFRESH_PERIOD}. */
    private int refreshesSinceTemperature = TEMPERATURE_REFRESH_PERIOD;
    private final double[] electrical = new double[3];
    private boolean integratedEncoderEnabled;
    private boolean absoluteEncoderEnabled;
    private double positionRotations;
    private double velocityRotationsPerSecond;
    private double absolutePositionRotations;
    private double absoluteVelocityRotationsPerSecond;
    private double appliedVoltage;
    private double supplyCurrentAmps;
    private double statorCurrentAmps;
    private double motorTemperatureCelsius = Double.NaN;
    private boolean fatalFault;

    /**
     * Creates a REV motor handle using real REVLib Spark controllers.
     *
     * @param device motor declaration
     * @param options REV options
     */
    public RevMotorHandle(MotorDevice device, RevMotorOptions options) {
        this(device, options, createController(device));
    }

    RevMotorHandle(MotorDevice device, RevMotorOptions options, SparkController controller) {
        this.device = Objects.requireNonNull(device, "device");
        this.options = options == null ? new RevMotorOptions() : options;
        this.controller = Objects.requireNonNull(controller, "controller");
        this.configuration = new ConfigurationQueue(device.defaultName(), this::stop);
        integratedEncoderEnabled = isBrushless(device);
    }

    @Override
    public MotorDevice device() {
        return device;
    }

    @Override
    public boolean supportsRuntimeConfiguration() {
        return true;
    }

    @Override public ConfigurationStatus configurationStatus() { return configuration.status(); }

    @Override
    public void applyRuntimeConfiguration(MotorRuntimeConfig configuration) {
        Objects.requireNonNull(configuration, "configuration");
        openLoop.invalidate();
        this.configuration.request(ConfigurationQueue.Key.RUNTIME, configuration, () -> {
            if (!controller.configureRuntime(configuration, isBrushless(device))) {
                throw new IllegalStateException("Failed to configure " + device.defaultName());
            }
        });
    }

    /**
     * Returns REV-specific options.
     *
     * @return options
     */
    public RevMotorOptions options() {
        return options;
    }

    @Override
    public void activate() {
        openLoop.invalidate();
        if (activated) {
            return;
        }
        if (device.follower() != null) {
            // A hardware follower receives its full activation config together with the follow
            // request in follow(). Configuring here too would write (and persist) twice per boot.
            activated = true;
            return;
        }
        configuration.request(ConfigurationQueue.Key.ACTIVATION, device,
                () -> configureOrRelease(() -> controller.configure(device, options), "configure"));
        activated = true;
    }

    /**
     * Runs on the ordered configuration worker. A failure remains attached to this controller for
     * bounded retry; close releases the native handle after any in-flight call has exited.
     */
    private void configureOrRelease(java.util.function.BooleanSupplier configure, String operation) {
        boolean ok;
        try {
            ok = configure.getAsBoolean();
        } catch (RuntimeException exception) {
            throw new IllegalStateException("Failed to " + operation + " " + device.defaultName()
                    + ": " + exception.getMessage(), exception);
        }
        if (!ok) {
            throw new IllegalStateException("Failed to " + operation + " " + device.defaultName());
        }
    }

    @Override
    public void refreshInputs() {
        configuration.retryFailed();
        if (configuration.status().failed()) throw new IllegalStateException(configuration.status().error());
        if (!configuration.status().ready()) return;
        if (!controller.isConnected()) {
            throw new IllegalStateException("REV motor is disconnected: " + device.defaultName());
        }
        if (integratedEncoderEnabled) {
            positionRotations = controller.positionRotations();
            velocityRotationsPerSecond = controller.velocityRotationsPerSecond();
        }
        if (absoluteEncoderEnabled) {
            absolutePositionRotations = controller.absolutePositionRotations();
            absoluteVelocityRotationsPerSecond = controller.absoluteVelocityRotationsPerSecond();
        }
        controller.electrical(electrical);
        appliedVoltage = electrical[0];
        supplyCurrentAmps = electrical[1];
        statorCurrentAmps = electrical[2];
        // Each SPARK read is a JNI call on the roboRIO; temperature moves over seconds, so it is read less often.
        if (++refreshesSinceTemperature >= TEMPERATURE_REFRESH_PERIOD) {
            refreshesSinceTemperature = 0;
            motorTemperatureCelsius = controller.motorTemperatureCelsius();
        }
        fatalFault = controller.fatalFault();
        inputsFresh = true;
    }

    @Override
    public void follow(MotorHandle leader, boolean inverted) {
        openLoop.invalidate();
        if (!(leader instanceof RevMotorHandle revLeader)) {
            throw new IllegalArgumentException("REV motor " + device.defaultName()
                    + " can only follow another REV motor.");
        }
        configuration.request(ConfigurationQueue.Key.FOLLOW, new FollowConfig(revLeader.device.id(), inverted),
                () -> configureOrRelease(
                        () -> controller.follow(device, options, revLeader.device.id(), inverted), "configure follower"));
    }

    /** Leaves follower mode uninverted (a SPARK ignores direct commands while it follows). */
    @Override
    public void stopFollowing() {
        openLoop.invalidate();
        configuration.request(ConfigurationQueue.Key.FOLLOW, "independent", () -> {
            if (!controller.stopFollowing(device.inverted(false), options)) {
                throw new IllegalStateException("Failed to leave follower mode for " + device.defaultName());
            }
        });
    }

    @Override
    public void setPercentOutput(double percent) {
        if (!configuration.status().ready()) return;
        double safe = clamp(percent);
        if (!openLoop.percent(safe)) return;
        try {
            controller.setPercent(safe);
        } catch (RuntimeException exception) {
            openLoop.invalidate();
            throw exception;
        }
    }

    @Override
    public void setVoltage(double volts) {
        if (!configuration.status().ready()) return;
        double safe = finiteOrZero(volts);
        if (!openLoop.voltage(safe)) return;
        try {
            controller.setVoltage(safe);
        } catch (RuntimeException exception) {
            openLoop.invalidate();
            throw exception;
        }
    }

    @Override
    public void setPositionTargetRotations(double rotations) {
        if (!configuration.status().ready()) return;
        openLoop.invalidate();
        controller.setPositionTarget(finiteOrZero(rotations));
    }

    @Override
    public void setPositionTargetRotations(double rotations, MotorClosedLoopRequest request) {
        throw unsupportedVoltageClosedLoop();
    }

    @Override
    public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
        if (!configuration.status().ready()) return;
        openLoop.invalidate();
        controller.setVelocityTarget(finiteOrZero(rotationsPerSecond));
    }

    @Override
    public void setVelocityTargetRotationsPerSecond(
            double rotationsPerSecond,
            MotorClosedLoopRequest request) {
        throw unsupportedVoltageClosedLoop();
    }

    @Override
    public MotorControlCapabilities controlCapabilities() {
        // SPARK PID gains produce duty cycle while Athena gains are defined in volts.
        // Keep Athena's voltage loop on the host until REV exposes voltage-output PID.
        return MotorControlCapabilities.OPEN_LOOP_ONLY;
    }

    @Override
    public void stop() {
        openLoop.invalidate();
        controller.stop();
    }

    @Override
    public double integratedPositionRotations() {
        requireIntegratedEncoder();
        ensureInputsFresh();
        return positionRotations;
    }

    @Override
    public double integratedVelocityRotationsPerSecond() {
        requireIntegratedEncoder();
        ensureInputsFresh();
        return velocityRotationsPerSecond;
    }

    @Override
    public void setIntegratedPositionRotations(double rotations) {
        double safeRotations = finiteOrZero(rotations);
        configuration.request(ConfigurationQueue.Key.POSITION, safeRotations, () -> {
            requireIntegratedEncoder();
            if (!controller.setSensorPosition(safeRotations)) {
                throw new IllegalStateException("Failed to set position for " + device.defaultName());
            }
            positionRotations = safeRotations;
            inputsFresh = true;
        });
    }

    @Override
    public boolean supportsIntegratedPositionSetting() {
        return integratedEncoderEnabled;
    }

    @Override
    public void enableIntegratedEncoder() {
        if (integratedEncoderEnabled) {
            return;
        }
        if (options.primaryEncoderCountsPerRevolution() <= 0) {
            throw new IllegalStateException("Brushed REV motor " + device.defaultName()
                    + " needs RevMotorOptions.primaryEncoderCountsPerRevolution(...) "
                    + "before its attached quadrature encoder can be used.");
        }
        configuration.request(ConfigurationQueue.Key.INTEGRATED_ENCODER, true, () -> {
            if (!controller.enableIntegratedEncoder(options)) {
                throw new IllegalStateException("Failed to configure primary encoder for " + device.defaultName());
            }
            integratedEncoderEnabled = true;
            inputsFresh = false;
        });
    }

    @Override
    public void enableAbsoluteEncoder() {
        if (absoluteEncoderEnabled) {
            return;
        }
        configuration.request(ConfigurationQueue.Key.ABSOLUTE_ENCODER, true, () -> {
            if (!controller.enableAbsoluteEncoder(options)) {
                throw new IllegalStateException("Failed to configure absolute encoder for " + device.defaultName());
            }
            absoluteEncoderEnabled = true;
            inputsFresh = false;
        });
    }

    @Override
    public double absolutePositionRotations() {
        requireAbsoluteEncoder();
        ensureInputsFresh();
        return absolutePositionRotations;
    }

    @Override
    public double absoluteVelocityRotationsPerSecond() {
        requireAbsoluteEncoder();
        ensureInputsFresh();
        return absoluteVelocityRotationsPerSecond;
    }

    @Override public double appliedVoltage() { ensureInputsFresh(); return appliedVoltage; }
    @Override public double supplyCurrentAmps() { ensureInputsFresh(); return supplyCurrentAmps; }
    @Override public double statorCurrentAmps() { ensureInputsFresh(); return statorCurrentAmps; }

    /** SPARK motor temperature from the controller's periodic status, refreshed with the input batch. */
    @Override
    public java.util.OptionalDouble temperatureCelsius() {
        ensureInputsFresh();
        return Double.isFinite(motorTemperatureCelsius)
                ? java.util.OptionalDouble.of(motorTemperatureCelsius)
                : java.util.OptionalDouble.empty();
    }

    /**
     * Reports a SPARK fault that stops it from driving the motor (gate driver, motor type mismatch, ESC EEPROM
     * or firmware), as captured by the latest {@link #refreshInputs()}. Transient faults (CAN, temperature,
     * sensor, other) and warnings such as overcurrent or brownout do not count.
     */
    @Override
    public boolean faulted() {
        return fatalFault || configuration.status().failed();
    }

    @Override
    public void close() {
        configuration.close(controller::close);
    }

    private record FollowConfig(int leaderId, boolean inverted) { }

    private void ensureInputsFresh() {
        if (!inputsFresh) {
            refreshInputs();
        }
    }

    private void requireIntegratedEncoder() {
        if (!integratedEncoderEnabled) {
            throw new UnsupportedOperationException("No primary encoder is configured for " + device.defaultName());
        }
    }

    private void requireAbsoluteEncoder() {
        if (!absoluteEncoderEnabled) {
            throw new UnsupportedOperationException("No absolute encoder is configured for " + device.defaultName());
        }
    }

    private UnsupportedOperationException unsupportedVoltageClosedLoop() {
        return new UnsupportedOperationException(
                "REV closed-loop gains use duty-cycle semantics; Athena voltage-gain requests must run in software for "
                        + device.defaultName());
    }

    private static SparkController createController(MotorDevice device) {
        if (!device.canbus().equalsIgnoreCase("rio")) {
            throw new IllegalArgumentException("REVLib 2026 only supports SPARK controllers on the roboRIO CAN bus; "
                    + device.defaultName() + " declared bus \"" + device.canbus() + "\".");
        }
        boolean brushless = isBrushless(device);
        MotorType motorType = brushless ? MotorType.kBrushless : MotorType.kBrushed;
        boolean flex = device.kind().controllerKind() == MotorControllerKinds.SPARK_FLEX
                || device.kind().key().startsWith("rev:spark-flex");
        SparkBase spark = flex
                        ? new SparkFlex(device.id(), motorType)
                        : new SparkMax(device.id(), motorType);
        return new RevSparkController(spark, flex);
    }

    private static boolean isBrushless(MotorDevice device) {
        return device.kind().technology() == MotorTechnology.BRUSHLESS
                || device.kind().technology() == MotorTechnology.UNKNOWN
                        && device.kind().key().endsWith("brushless");
    }

    private static double clamp(double value) {
        double finite = finiteOrZero(value);
        return Math.max(-1.0, Math.min(1.0, finite));
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    /** Converts a typed limit to the whole amps the SPARK API takes. */
    private static int wholeAmps(Current limit) {
        return (int) Math.round(limit.in(Amps));
    }

    /** Converts a typed period to the whole milliseconds the SPARK API takes. */
    private static int wholeMillis(Time period) {
        return (int) Math.round(period.in(Milliseconds));
    }

    static int effectiveSmartCurrentLimit(MotorDevice device, RevMotorOptions options) {
        int declared = declaredSmartCurrentLimit(device, options);
        if (declared > 0 || (options != null && options.preserveDeviceCurrentLimit())) {
            return declared;
        }
        return defaultSmartCurrentLimit(device);
    }

    private static int declaredSmartCurrentLimit(MotorDevice device, RevMotorOptions options) {
        int vendorLimit = options == null ? 0 : wholeAmps(options.smartCurrentLimit());
        if (vendorLimit > 0) {
            return vendorLimit;
        }
        // SPARK's limiter regulates phase current. An explicit battery-side limit belongs
        // to Athena's power pass, not this field; the portable currentLimit still maps to phase.
        return wholeAmps(device.currentLimits().stator());
    }

    /**
     * Returns the smart current limit used when robot code declares none.
     *
     * <p>Leaving the limit unset previously kept whatever the SPARK had stored (80 A from the
     * factory), so several stalled NEOs at enable could brown out the roboRIO. Brushed motors keep
     * the device value because their secondary limit is a hard chopper with no safe generic default.</p>
     */
    static int defaultSmartCurrentLimit(MotorDevice device) {
        if (!isBrushless(device)) {
            return 0;
        }
        if (device.kind().motorKind() instanceof ca.frc6390.athena.api.hardware.MotorKinds motor) {
            return switch (motor) {
                case NEO_550 -> DEFAULT_NEO_550_CURRENT_LIMIT_AMPS;
                case NEO_VORTEX -> DEFAULT_NEO_VORTEX_CURRENT_LIMIT_AMPS;
                default -> DEFAULT_NEO_CURRENT_LIMIT_AMPS;
            };
        }
        return DEFAULT_NEO_CURRENT_LIMIT_AMPS;
    }

    static SparkBaseConfig activationConfig(MotorDevice device, RevMotorOptions options, boolean flex) {
        SparkBaseConfig config = flex ? new SparkFlexConfig() : new SparkMaxConfig();
        config.idleMode(device.neutralMode() == MotorNeutralMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
        config.inverted(device.isInverted());
        int currentLimit = effectiveSmartCurrentLimit(device, options);
        if (currentLimit > 0) {
            if (isBrushless(device)) {
                config.smartCurrentLimit(currentLimit);
            } else {
                config.secondaryCurrentLimit(currentLimit);
            }
        }
        if (options.openLoopRampConfigured()) {
            config.openLoopRampRate(options.openLoopRamp().in(Seconds));
        }
        if (options.closedLoopRampConfigured()) {
            config.closedLoopRampRate(options.closedLoopRamp().in(Seconds));
        }
        if (options.voltageCompensationConfigured()) {
            if (options.voltageCompensation().in(Volts) > 0.0) {
                config.voltageCompensation(options.voltageCompensation().in(Volts));
            } else {
                config.disableVoltageCompensation();
            }
        }
        if (isBrushless(device) || options.primaryEncoderCountsPerRevolution() > 0) {
            configurePrimaryEncoder(config, options);
        }
        configureLimits(config, options, flex);
        if (wholeMillis(options.telemetrySignalPeriod()) > 0) {
            config.signals.appliedOutputPeriodMs(wholeMillis(options.telemetrySignalPeriod()))
                    .busVoltagePeriodMs(wholeMillis(options.telemetrySignalPeriod()))
                    .outputCurrentPeriodMs(wholeMillis(options.telemetrySignalPeriod()));
        }
        config.disableFollowerMode();
        return config;
    }

    static SparkBaseConfig runtimeConfig(MotorRuntimeConfig configuration, boolean brushless, boolean flex) {
        SparkBaseConfig config = flex ? new SparkFlexConfig() : new SparkMaxConfig();
        config.idleMode(configuration.neutralMode() == MotorNeutralMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
        config.inverted(configuration.inverted());
        int phase = configuration.statorCurrentLimitAmps();
        if (phase <= 0 && brushless) {
            phase = flex ? DEFAULT_NEO_VORTEX_CURRENT_LIMIT_AMPS : DEFAULT_NEO_CURRENT_LIMIT_AMPS;
        }
        if (phase > 0) {
            if (brushless) config.smartCurrentLimit(phase);
            else config.secondaryCurrentLimit(phase);
        }
        return config;
    }

    private static void configureLimits(SparkBaseConfig config, RevMotorOptions options, boolean flex) {
        if (options.forwardSoftLimitConfigured()) {
            boolean enabled = Double.isFinite(options.forwardSoftLimit().in(Rotations));
            config.softLimit.forwardSoftLimitEnabled(enabled);
            if (enabled) {
                config.softLimit.forwardSoftLimit(options.forwardSoftLimit().in(Rotations));
            }
        }
        if (options.reverseSoftLimitConfigured()) {
            boolean enabled = Double.isFinite(options.reverseSoftLimit().in(Rotations));
            config.softLimit.reverseSoftLimitEnabled(enabled);
            if (enabled) {
                config.softLimit.reverseSoftLimit(options.reverseSoftLimit().in(Rotations));
            }
        }
        if (!flex && (options.forwardLimitSwitchEnabled() || options.reverseLimitSwitchEnabled())) {
            config.limitSwitch.setSparkMaxDataPortConfig();
        }
        if (options.forwardLimitSwitchConfigured()) {
            config.limitSwitch
                    .forwardLimitSwitchType(options.forwardLimitSwitchNormallyClosed()
                            ? Type.kNormallyClosed : Type.kNormallyOpen)
                    .forwardLimitSwitchTriggerBehavior(options.forwardLimitSwitchEnabled()
                            ? Behavior.kStopMovingMotor : Behavior.kKeepMovingMotor);
        }
        if (options.reverseLimitSwitchConfigured()) {
            config.limitSwitch
                    .reverseLimitSwitchType(options.reverseLimitSwitchNormallyClosed()
                            ? Type.kNormallyClosed : Type.kNormallyOpen)
                    .reverseLimitSwitchTriggerBehavior(options.reverseLimitSwitchEnabled()
                            ? Behavior.kStopMovingMotor : Behavior.kKeepMovingMotor);
        }
    }

    private static void configurePrimaryEncoder(SparkBaseConfig config, RevMotorOptions options) {
        if (options.primaryEncoderCountsPerRevolution() > 0) {
            config.encoder.countsPerRevolution(options.primaryEncoderCountsPerRevolution());
        }
        config.signals.primaryEncoderPositionAlwaysOn(true)
                .primaryEncoderVelocityAlwaysOn(true);
        if (wholeMillis(options.primaryEncoderSignalPeriod()) > 0) {
            config.signals.primaryEncoderPositionPeriodMs(wholeMillis(options.primaryEncoderSignalPeriod()))
                    .primaryEncoderVelocityPeriodMs(wholeMillis(options.primaryEncoderSignalPeriod()));
        }
    }

    static SparkBaseConfig absoluteEncoderConfig(RevMotorOptions options, boolean flex) {
        SparkBaseConfig config = flex ? new SparkFlexConfig() : new SparkMaxConfig();
        if (!flex) {
            config.absoluteEncoder.setSparkMaxDataPortConfig();
        }
        switch (options.absoluteEncoderProfile()) {
            case REV_THROUGH_BORE_V1 -> config.absoluteEncoder.startPulseUs(1.0).endPulseUs(1024.0);
            case REV_THROUGH_BORE_V2 -> config.absoluteEncoder.startPulseUs(3.884).endPulseUs(998.06);
            case GENERIC -> { }
        }
        if (options.absoluteEncoderAverageDepth() > 0) {
            config.absoluteEncoder.averageDepth(options.absoluteEncoderAverageDepth());
        }
        config.signals.absoluteEncoderPositionAlwaysOn(true)
                .absoluteEncoderVelocityAlwaysOn(true);
        if (wholeMillis(options.absoluteEncoderSignalPeriod()) > 0) {
            config.signals.absoluteEncoderPositionPeriodMs(wholeMillis(options.absoluteEncoderSignalPeriod()))
                    .absoluteEncoderVelocityPeriodMs(wholeMillis(options.absoluteEncoderSignalPeriod()));
        }
        return config;
    }

    interface SparkController {
        default boolean isConnected() { return true; }

        boolean configure(MotorDevice device, RevMotorOptions options);

        default boolean configureRuntime(MotorRuntimeConfig configuration, boolean brushless) {
            return false;
        }

        void setPercent(double percent);

        void setVoltage(double volts);

        void setPositionTarget(double rotations);

        void setVelocityTarget(double rotationsPerSecond);

        default boolean setSensorPosition(double rotations) {
            throw new UnsupportedOperationException("Integrated encoder position cannot be set.");
        }

        default void setPositionTarget(double rotations, int slot, double arbitraryFeedforwardVolts) {
            setPositionTarget(rotations);
        }

        default void setVelocityTarget(double rotationsPerSecond, int slot, double arbitraryFeedforwardVolts) {
            setVelocityTarget(rotationsPerSecond);
        }

        default boolean follow(
                MotorDevice device,
                RevMotorOptions options,
                int leaderId,
                boolean inverted) {
            throw new UnsupportedOperationException("Hardware following is unavailable.");
        }

        default boolean stopFollowing(MotorDevice device, RevMotorOptions options) { return true; }

        default boolean enableIntegratedEncoder(RevMotorOptions options) { return true; }

        default boolean enableAbsoluteEncoder(RevMotorOptions options) { return true; }

        void stop();

        double positionRotations();

        double velocityRotationsPerSecond();

        double absolutePositionRotations();

        double absoluteVelocityRotationsPerSecond();

        default double appliedVoltage() { return 0.0; }

        default double outputCurrentAmps() { return 0.0; }

        default double supplyCurrentAmps() { return outputCurrentAmps(); }

        default double statorCurrentAmps() { return outputCurrentAmps(); }

        /**
         * Reads applied voltage, supply current and stator current into {@code out[0..2]}. A SPARK derives all three
         * from applied output, bus voltage and output current, so it reads each of those once.
         */
        default void electrical(double[] out) {
            out[0] = appliedVoltage();
            out[1] = supplyCurrentAmps();
            out[2] = statorCurrentAmps();
        }

        /** Motor temperature in Celsius, or NaN when the controller does not report one. */
        default double motorTemperatureCelsius() { return Double.NaN; }

        /** Whether an active fault stops the controller from driving the motor. Reads cached status only. */
        default boolean fatalFault() { return false; }

        default void close() {}
    }

    /** Refreshes between motor temperature reads (200 ms at 50 Hz); the first refresh always reads it. */
    static final int TEMPERATURE_REFRESH_PERIOD = 10;

    /** {@link SparkBase.Faults} bits that stop a SPARK from driving: motor type, gate driver, ESC EEPROM, firmware. */
    static final int FATAL_FAULT_BITS = 0x02 | 0x20 | 0x40 | 0x80;

    private static final class RevSparkController implements SparkController {
        private final SparkBase spark;
        private final boolean flex;

        private RevSparkController(SparkBase spark, boolean flex) {
            this.spark = spark;
            this.flex = flex;
        }

        @Override
        public boolean configure(MotorDevice device, RevMotorOptions options) {
            SparkBaseConfig config = activationConfig(device, options, flex);
            return apply(config, options.resetSafeParameters(), options.persistParameters());
        }

        @Override
        public boolean isConnected() {
            return spark.getFirmwareVersion() != 0;
        }

        @Override
        public boolean configureRuntime(MotorRuntimeConfig configuration, boolean brushless) {
            return apply(runtimeConfig(configuration, brushless, flex), false, false);
        }

        @Override
        public boolean follow(
                MotorDevice device,
                RevMotorOptions options,
                int leaderId,
                boolean inverted) {
            SparkBaseConfig config = activationConfig(device, options, flex);
            config.follow(leaderId, inverted);
            return apply(config, options.resetSafeParameters(), options.persistParameters());
        }

        @Override
        public boolean stopFollowing(MotorDevice device, RevMotorOptions options) {
            // The activation config disables follower mode. Not persisted: flash keeps the declared follow setup.
            return apply(activationConfig(device, options, flex), false, false);
        }

        @Override
        public boolean enableIntegratedEncoder(RevMotorOptions options) {
            SparkBaseConfig config = flex ? new SparkFlexConfig() : new SparkMaxConfig();
            configurePrimaryEncoder(config, options);
            return apply(config, false, options.persistParameters());
        }

        @Override
        public boolean enableAbsoluteEncoder(RevMotorOptions options) {
            SparkBaseConfig config = absoluteEncoderConfig(options, flex);
            return apply(config, false, options.persistParameters());
        }

        private boolean apply(SparkBaseConfig config, boolean reset, boolean persist) {
            ResetMode resetMode = reset ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters;
            REVLibError status = spark.configure(
                    config,
                    resetMode,
                    persist ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters);
            if (persist && status == REVLibError.kCannotPersistParametersWhileEnabled) {
                // A SPARK that (re)binds while the robot is enabled (late power-up, breaker trip)
                // cannot burn flash. Apply the configuration for this power cycle instead of failing
                // and retrying the whole bind every few seconds.
                status = spark.configure(config, resetMode, PersistMode.kNoPersistParameters);
            }
            return status == REVLibError.kOk;
        }

        @Override
        public void setPercent(double percent) {
            spark.set(percent);
        }

        @Override
        public void setVoltage(double volts) {
            spark.setVoltage(volts);
        }

        @Override
        public void setPositionTarget(double rotations) {
            spark.getClosedLoopController().setSetpoint(rotations, ControlType.kPosition);
        }

        @Override
        public void setVelocityTarget(double rotationsPerSecond) {
            spark.getClosedLoopController().setSetpoint(rotationsPerSecond * 60.0, ControlType.kVelocity);
        }

        @Override
        public boolean setSensorPosition(double rotations) {
            return spark.getEncoder().setPosition(rotations) == REVLibError.kOk;
        }

        @Override
        public void setPositionTarget(double rotations, int slot, double arbitraryFeedforwardVolts) {
            spark.getClosedLoopController().setSetpoint(
                    rotations,
                    ControlType.kPosition,
                    closedLoopSlot(slot),
                    arbitraryFeedforwardVolts,
                    ArbFFUnits.kVoltage);
        }

        @Override
        public void setVelocityTarget(double rotationsPerSecond, int slot, double arbitraryFeedforwardVolts) {
            spark.getClosedLoopController().setSetpoint(
                    rotationsPerSecond * 60.0,
                    ControlType.kVelocity,
                    closedLoopSlot(slot),
                    arbitraryFeedforwardVolts,
                    ArbFFUnits.kVoltage);
        }

        @Override
        public void stop() {
            spark.stopMotor();
        }

        @Override
        public double positionRotations() {
            return encoder().getPosition();
        }

        @Override
        public double velocityRotationsPerSecond() {
            return encoder().getVelocity() / 60.0;
        }

        @Override
        public double absolutePositionRotations() {
            return absoluteEncoder().getPosition();
        }

        @Override
        public double absoluteVelocityRotationsPerSecond() {
            return absoluteEncoder().getVelocity() / 60.0;
        }

        @Override public double appliedVoltage() { return spark.getAppliedOutput() * spark.getBusVoltage(); }

        @Override public double outputCurrentAmps() { return spark.getOutputCurrent(); }

        @Override public double supplyCurrentAmps() {
            // SPARK reports motor phase current. For a PWM controller, battery-side
            // current is approximately phase current multiplied by duty cycle.
            return Math.abs(spark.getAppliedOutput()) * spark.getOutputCurrent();
        }

        @Override public double statorCurrentAmps() { return spark.getOutputCurrent(); }

        @Override
        public void electrical(double[] out) {
            double appliedOutput = spark.getAppliedOutput();
            double outputCurrent = spark.getOutputCurrent();
            out[0] = appliedOutput * spark.getBusVoltage();
            out[1] = Math.abs(appliedOutput) * outputCurrent;
            out[2] = outputCurrent;
        }

        @Override public double motorTemperatureCelsius() { return spark.getMotorTemperature(); }

        @Override
        public boolean fatalFault() {
            // hasActiveFault() reads the cached status frame without allocating; getFaults() allocates, so it
            // only runs on the rare cycle where some fault is active.
            return spark.hasActiveFault() && (spark.getFaults().rawBits & FATAL_FAULT_BITS) != 0;
        }

        @Override public void close() { spark.close(); }

        private RelativeEncoder encoder() {
            return spark.getEncoder();
        }

        private AbsoluteEncoder absoluteEncoder() {
            return spark.getAbsoluteEncoder();
        }

        private static ClosedLoopSlot closedLoopSlot(int slot) {
            return ClosedLoopSlot.fromInt(Math.max(0, Math.min(3, slot)));
        }
    }
}
