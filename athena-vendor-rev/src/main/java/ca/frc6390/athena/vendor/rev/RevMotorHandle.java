package ca.frc6390.athena.vendor.rev;

import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.api.hardware.MotorControllerKinds;
import ca.frc6390.athena.api.hardware.MotorTechnology;
import ca.frc6390.athena.hardware.backend.MotorHandle;
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
    private final MotorDevice device;
    private final RevMotorOptions options;
    private final SparkController controller;
    private boolean activated;
    private boolean inputsFresh;
    private boolean integratedEncoderEnabled;
    private boolean absoluteEncoderEnabled;
    private double positionRotations;
    private double velocityRotationsPerSecond;
    private double absolutePositionRotations;
    private double absoluteVelocityRotationsPerSecond;
    private double appliedVoltage;
    private double supplyCurrentAmps;
    private double statorCurrentAmps;

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
        integratedEncoderEnabled = isBrushless(device);
    }

    @Override
    public MotorDevice device() {
        return device;
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
        if (!activated) {
            if (!controller.configure(device, options)) {
                throw new IllegalStateException("Failed to configure " + device.defaultName());
            }
            activated = true;
        }
    }

    @Override
    public void refreshInputs() {
        if (integratedEncoderEnabled) {
            positionRotations = controller.positionRotations();
            velocityRotationsPerSecond = controller.velocityRotationsPerSecond();
        }
        if (absoluteEncoderEnabled) {
            absolutePositionRotations = controller.absolutePositionRotations();
            absoluteVelocityRotationsPerSecond = controller.absoluteVelocityRotationsPerSecond();
        }
        appliedVoltage = controller.appliedVoltage();
        supplyCurrentAmps = controller.supplyCurrentAmps();
        statorCurrentAmps = controller.statorCurrentAmps();
        inputsFresh = true;
    }

    @Override
    public void follow(MotorHandle leader, boolean inverted) {
        if (!(leader instanceof RevMotorHandle revLeader)) {
            throw new IllegalArgumentException("REV motor " + device.defaultName()
                    + " can only follow another REV motor.");
        }
        if (!controller.follow(device, options, revLeader.device.id(), inverted)) {
            throw new IllegalStateException("Failed to configure follower " + device.defaultName());
        }
    }

    @Override
    public void setPercentOutput(double percent) {
        controller.setPercent(clamp(percent));
    }

    @Override
    public void setVoltage(double volts) {
        controller.setVoltage(finiteOrZero(volts));
    }

    @Override
    public void setPositionTargetRotations(double rotations) {
        controller.setPositionTarget(finiteOrZero(rotations));
    }

    @Override
    public void setPositionTargetRotations(double rotations, MotorClosedLoopRequest request) {
        throw unsupportedVoltageClosedLoop();
    }

    @Override
    public void setVelocityTargetRotationsPerSecond(double rotationsPerSecond) {
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
        requireIntegratedEncoder();
        double safeRotations = finiteOrZero(rotations);
        if (!controller.setSensorPosition(safeRotations)) {
            throw new IllegalStateException("Failed to set position for " + device.defaultName());
        }
        positionRotations = safeRotations;
        inputsFresh = true;
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
        if (!controller.enableIntegratedEncoder(options)) {
            throw new IllegalStateException("Failed to configure primary encoder for " + device.defaultName());
        }
        integratedEncoderEnabled = true;
        inputsFresh = false;
    }

    @Override
    public void enableAbsoluteEncoder() {
        if (absoluteEncoderEnabled) {
            return;
        }
        if (!controller.enableAbsoluteEncoder(options)) {
            throw new IllegalStateException("Failed to configure absolute encoder for " + device.defaultName());
        }
        absoluteEncoderEnabled = true;
        inputsFresh = false;
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

    @Override
    public void close() {
        controller.close();
    }

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

    static int effectiveSmartCurrentLimit(MotorDevice device, RevMotorOptions options) {
        int vendorLimit = options == null ? 0 : options.smartCurrentLimitAmps();
        if (vendorLimit > 0) {
            return vendorLimit;
        }
        int supplyLimit = device.supplyCurrentLimitAmps() > 0
                ? device.supplyCurrentLimitAmps()
                : device.currentLimitAmps();
        int statorLimit = device.statorCurrentLimitAmps();
        if (supplyLimit <= 0) {
            return statorLimit;
        }
        if (statorLimit <= 0) {
            return supplyLimit;
        }
        // REV only exposes a phase-current limiter. The lower requested ceiling is
        // the conservative mapping when Athena declares both current domains.
        return Math.min(supplyLimit, statorLimit);
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
            config.openLoopRampRate(options.openLoopRampSeconds());
        }
        if (options.closedLoopRampConfigured()) {
            config.closedLoopRampRate(options.closedLoopRampSeconds());
        }
        if (options.voltageCompensationConfigured()) {
            if (options.voltageCompensationVolts() > 0.0) {
                config.voltageCompensation(options.voltageCompensationVolts());
            } else {
                config.disableVoltageCompensation();
            }
        }
        if (isBrushless(device) || options.primaryEncoderCountsPerRevolution() > 0) {
            configurePrimaryEncoder(config, options);
        }
        configureLimits(config, options, flex);
        if (options.telemetrySignalPeriodMs() > 0) {
            config.signals.appliedOutputPeriodMs(options.telemetrySignalPeriodMs())
                    .busVoltagePeriodMs(options.telemetrySignalPeriodMs())
                    .outputCurrentPeriodMs(options.telemetrySignalPeriodMs());
        }
        config.disableFollowerMode();
        return config;
    }

    private static void configureLimits(SparkBaseConfig config, RevMotorOptions options, boolean flex) {
        if (options.forwardSoftLimitConfigured()) {
            boolean enabled = Double.isFinite(options.forwardSoftLimitRotations());
            config.softLimit.forwardSoftLimitEnabled(enabled);
            if (enabled) {
                config.softLimit.forwardSoftLimit(options.forwardSoftLimitRotations());
            }
        }
        if (options.reverseSoftLimitConfigured()) {
            boolean enabled = Double.isFinite(options.reverseSoftLimitRotations());
            config.softLimit.reverseSoftLimitEnabled(enabled);
            if (enabled) {
                config.softLimit.reverseSoftLimit(options.reverseSoftLimitRotations());
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
        if (options.primaryEncoderSignalPeriodMs() > 0) {
            config.signals.primaryEncoderPositionPeriodMs(options.primaryEncoderSignalPeriodMs())
                    .primaryEncoderVelocityPeriodMs(options.primaryEncoderSignalPeriodMs());
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
        if (options.absoluteEncoderSignalPeriodMs() > 0) {
            config.signals.absoluteEncoderPositionPeriodMs(options.absoluteEncoderSignalPeriodMs())
                    .absoluteEncoderVelocityPeriodMs(options.absoluteEncoderSignalPeriodMs());
        }
        return config;
    }

    interface SparkController {
        boolean configure(MotorDevice device, RevMotorOptions options);

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

        default void close() {}
    }

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
            return spark.configure(
                            config,
                            reset ? ResetMode.kResetSafeParameters : ResetMode.kNoResetSafeParameters,
                            persist ? PersistMode.kPersistParameters : PersistMode.kNoPersistParameters)
                    == REVLibError.kOk;
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
