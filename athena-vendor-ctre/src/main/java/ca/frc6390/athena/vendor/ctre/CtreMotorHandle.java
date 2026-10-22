package ca.frc6390.athena.vendor.ctre;

import static edu.wpi.first.units.Units.Amps;

import ca.frc6390.athena.hardware.backend.FocPolicy;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopConfig;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.ConfigurationQueue;
import ca.frc6390.athena.hardware.backend.ConfigurationStatus;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.backend.OpenLoopCommandFilter;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import ca.frc6390.athena.api.hardware.MotorControllerKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.MotorArrangementValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Objects;

/**
 * CTRE TalonFX-family motor handle backed by Phoenix 6.
 */
public final class CtreMotorHandle implements MotorHandle, AutoCloseable {
    private final MotorDevice device;
    private final CtreMotorOptions options;
    private final TalonController controller;
    private final ConfigurationQueue configuration;
    private boolean activated;
    private boolean inputsFresh;
    private final OpenLoopCommandFilter openLoop = new OpenLoopCommandFilter();
    private double positionRotations;
    private double velocityRotationsPerSecond;
    private double appliedVoltage;
    private double supplyCurrentAmps;
    private double statorCurrentAmps;
    private boolean fatalFault;
    private boolean focDisabledAfterLicenseFault;

    /**
     * Creates a CTRE motor handle using a real Phoenix 6 TalonFX-family controller.
     *
     * @param device motor declaration
     * @param options CTRE options
     */
    public CtreMotorHandle(MotorDevice device, CtreMotorOptions options) {
        this(device, options, device.kind().controllerKind() == MotorControllerKinds.TALON_FXS
                || device.kind().key().startsWith("ctre:talon-fxs/")
                ? new PhoenixTalonFxsController(device)
                : new PhoenixTalonController(device));
    }

    CtreMotorHandle(MotorDevice device, CtreMotorOptions options, TalonController controller) {
        this.device = Objects.requireNonNull(device, "device");
        this.options = options == null ? new CtreMotorOptions() : options;
        this.controller = Objects.requireNonNull(controller, "controller");
        this.configuration = new ConfigurationQueue(device.defaultName(), this::stop);
    }

    @Override
    public MotorDevice device() {
        return device;
    }

    @Override
    public boolean supportsRuntimeConfiguration() {
        return true;
    }

    @Override
    public boolean enforcesSupplyCurrentLimit() {
        return true;
    }

    @Override public ConfigurationStatus configurationStatus() { return configuration.status(); }

    @Override
    public void applyRuntimeConfiguration(MotorRuntimeConfig configuration) {
        Objects.requireNonNull(configuration, "configuration");
        openLoop.invalidate();
        this.configuration.request(ConfigurationQueue.Key.RUNTIME, configuration, () -> applyConfigurationNow(configuration));
    }

    private void applyConfigurationNow(MotorRuntimeConfig configuration) {
        if (!controller.configureOutput(
                configuration.neutralMode() == MotorNeutralMode.BRAKE,
                device.follower() == null && configuration.inverted())) {
            throw new IllegalStateException("Failed to configure output for " + device.defaultName());
        }
        if (!controller.configureCurrentLimits(
                configuration.supplyCurrentLimitAmps(),
                configuration.statorCurrentLimitAmps(),
                wholeAmps(options.torqueCurrentLimit()))) {
            throw new IllegalStateException("Failed to configure current limits for " + device.defaultName());
        }
    }

    /** Converts a typed limit to the whole amps the CTRE controller interface takes. */
    private static int wholeAmps(Current limit) {
        return (int) Math.round(limit.in(Amps));
    }

    /**
     * Returns CTRE-specific options.
     *
     * @return options
     */
    public CtreMotorOptions options() {
        return options;
    }

    @Override
    public void activate() {
        openLoop.invalidate();
        if (!activated) {
            activated = true;
            configuration.request(ConfigurationQueue.Key.ACTIVATION, device, this::activateNow);
        }
    }

    private void activateNow() {
            if (!controller.configureOutput(
                    device.neutralMode() == MotorNeutralMode.BRAKE,
                    device.follower() == null && device.isInverted())) {
                throw new IllegalStateException("Failed to configure output for " + device.defaultName());
            }
            int supplyLimit = wholeAmps(options.supplyCurrentLimit()) > 0
                    ? wholeAmps(options.supplyCurrentLimit())
                    : wholeAmps(device.supplyCurrentLimit()) > 0
                            ? wholeAmps(device.supplyCurrentLimit())
                            : wholeAmps(device.currentLimit());
            int statorLimit = wholeAmps(options.statorCurrentLimit()) > 0
                    ? wholeAmps(options.statorCurrentLimit())
                    : wholeAmps(device.statorCurrentLimit());
            if (!controller.configureCurrentLimits(
                    supplyLimit,
                    statorLimit,
                    wholeAmps(options.torqueCurrentLimit()))) {
                throw new IllegalStateException("Failed to configure current limits for " + device.defaultName());
            }
    }

    @Override
    public void refreshInputs() {
        configuration.retryFailed();
        if (configuration.status().failed()) throw new IllegalStateException(configuration.status().error());
        if (!configuration.status().ready()) return;
        // The connection check reads the frame timestamps of this batch, so refresh first.
        controller.refreshSignals();
        if (!controller.isConnected()) {
            throw new IllegalStateException("CTRE motor is disconnected: " + device.defaultName());
        }
        positionRotations = controller.positionRotations();
        velocityRotationsPerSecond = controller.velocityRotationsPerSecond();
        appliedVoltage = controller.appliedVoltage();
        supplyCurrentAmps = controller.supplyCurrentAmps();
        statorCurrentAmps = controller.statorCurrentAmps();
        fatalFault = controller.fatalFault();
        inputsFresh = true;
    }

    /**
     * Reports a Talon fault that stops it from driving the motor, as captured by the latest
     * {@link #refreshInputs()}: a hardware fault, plus on a TalonFXS a bridge short, a drive disabled by its
     * Hall sensor, or no motor arrangement selected. Limits, temperature, brownout and supply faults do not count.
     */
    @Override
    public boolean faulted() {
        return fatalFault || configuration.status().failed();
    }

    @Override
    public void follow(MotorHandle leader, boolean inverted) {
        openLoop.invalidate();
        if (!(leader instanceof CtreMotorHandle ctreLeader)) {
            throw new IllegalArgumentException("CTRE motor " + device.defaultName()
                    + " can only follow another CTRE motor.");
        }
        String followerBus = normalizedBus(device.canbus());
        String leaderBus = normalizedBus(ctreLeader.device.canbus());
        if (!followerBus.equalsIgnoreCase(leaderBus)) {
            // Phoenix Follower requests address the leader by CAN ID only, on the follower's own bus.
            // Across buses the follower would silently do nothing or follow an unrelated device
            // that happens to share the ID.
            throw new IllegalStateException("CTRE motor " + device.defaultName() + " on CAN bus '" + followerBus
                    + "' cannot hardware-follow " + ctreLeader.device.defaultName() + " on CAN bus '" + leaderBus
                    + "'. Phoenix followers must share the leader's bus; move the devices to one bus or use"
                    + " software following.");
        }
        configuration.request(ConfigurationQueue.Key.FOLLOW, new FollowConfig(ctreLeader.device.id(), inverted),
                () -> controller.follow(ctreLeader.device.id(), inverted));
    }

    static String normalizedBus(String canbus) {
        return canbus == null || canbus.isBlank() ? "rio" : canbus.trim();
    }

    @Override
    public void stopFollowing() {
        openLoop.invalidate();
        configuration.request(ConfigurationQueue.Key.FOLLOW, "independent", controller::stop);
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
        openLoop.invalidate();
        MotorClosedLoopRequest safeRequest = safeRequest(request);
        if (!applyClosedLoopConfig(safeRequest.config())) return;
        FocPolicy focPolicy = effectiveFocPolicy(safeRequest.config());
        boolean focRequested = enableFoc(focPolicy);
        boolean licenseFault = controller.setPositionTarget(
                finiteOrZero(rotations),
                safeRequest.config().slot(),
                finiteOrZero(safeRequest.arbitraryFeedforwardVolts()),
                focRequested);
        if (licenseFault && focRequested) {
            if (focPolicy == FocPolicy.REQUIRE) {
                throw new IllegalStateException("FOC is required but not licensed for " + device.defaultName());
            }
            if (focPolicy == FocPolicy.ENABLE_IF_AVAILABLE) {
                focDisabledAfterLicenseFault = true;
                controller.setPositionTarget(
                        finiteOrZero(rotations),
                        safeRequest.config().slot(),
                        finiteOrZero(safeRequest.arbitraryFeedforwardVolts()),
                        false);
            }
        }
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
        openLoop.invalidate();
        MotorClosedLoopRequest safeRequest = safeRequest(request);
        if (!applyClosedLoopConfig(safeRequest.config())) return;
        FocPolicy focPolicy = effectiveFocPolicy(safeRequest.config());
        boolean focRequested = enableFoc(focPolicy);
        boolean licenseFault = controller.setVelocityTarget(
                finiteOrZero(rotationsPerSecond),
                safeRequest.config().slot(),
                finiteOrZero(safeRequest.arbitraryFeedforwardVolts()),
                focRequested);
        if (licenseFault && focRequested) {
            if (focPolicy == FocPolicy.REQUIRE) {
                throw new IllegalStateException("FOC is required but not licensed for " + device.defaultName());
            }
            if (focPolicy == FocPolicy.ENABLE_IF_AVAILABLE) {
                focDisabledAfterLicenseFault = true;
                controller.setVelocityTarget(
                        finiteOrZero(rotationsPerSecond),
                        safeRequest.config().slot(),
                        finiteOrZero(safeRequest.arbitraryFeedforwardVolts()),
                        false);
            }
        }
    }

    @Override
    public MotorControlCapabilities controlCapabilities() {
        return new MotorControlCapabilities(true, true, true, true, true, false, 3);
    }

    @Override
    public void stop() {
        openLoop.invalidate();
        controller.stop();
    }

    @Override
    public double integratedPositionRotations() {
        ensureInputsFresh();
        return positionRotations;
    }

    @Override
    public double integratedVelocityRotationsPerSecond() {
        ensureInputsFresh();
        return velocityRotationsPerSecond;
    }

    @Override
    public double appliedVoltage() { ensureInputsFresh(); return appliedVoltage; }

    @Override
    public double supplyCurrentAmps() { ensureInputsFresh(); return supplyCurrentAmps; }

    @Override
    public double statorCurrentAmps() { ensureInputsFresh(); return statorCurrentAmps; }

    /** Device temperature from the TalonFX/TalonFXS DeviceTemp signal, refreshed with the batch. */
    @Override
    public java.util.OptionalDouble temperatureCelsius() { ensureInputsFresh(); return controller.temperatureCelsius(); }

    @Override
    public void setIntegratedPositionRotations(double rotations) {
        double safeRotations = finiteOrZero(rotations);
        configuration.request(ConfigurationQueue.Key.POSITION, safeRotations, () -> {
            controller.setSensorPosition(safeRotations);
            positionRotations = safeRotations;
            inputsFresh = true;
        });
    }

    @Override
    public boolean supportsIntegratedPositionSetting() {
        return true;
    }

    private void ensureInputsFresh() {
        if (!inputsFresh) {
            refreshInputs();
        }
    }

    private MotorClosedLoopRequest safeRequest(MotorClosedLoopRequest request) {
        return request == null ? MotorClosedLoopRequest.device(MotorClosedLoopConfig.empty()) : request;
    }

    private boolean applyClosedLoopConfig(MotorClosedLoopConfig config) {
        return configuration.request(ConfigurationQueue.Key.SLOT, config, () -> {
            if (!controller.configureSlot(config)) {
                throw new IllegalStateException("Failed to configure closed-loop slot for " + device.defaultName());
            }
        });
    }

    @Override public void close() { configuration.close(controller::close); }

    private record FollowConfig(int leaderId, boolean inverted) { }

    private FocPolicy effectiveFocPolicy(MotorClosedLoopConfig config) {
        return options.focPolicy() == FocPolicy.DISABLED ? config.focPolicy() : options.focPolicy();
    }

    private boolean enableFoc(FocPolicy policy) {
        return policy != FocPolicy.DISABLED && !focDisabledAfterLicenseFault;
    }

    private static double clamp(double value) {
        double finite = finiteOrZero(value);
        return Math.max(-1.0, Math.min(1.0, finite));
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    interface TalonController {
        default void close() { }
        default boolean isConnected() { return true; }

        void setPercent(double percent);

        void setVoltage(double volts);

        void setPositionTarget(double rotations);

        void setVelocityTarget(double rotationsPerSecond);

        default void setSensorPosition(double rotations) {
            throw new UnsupportedOperationException("Integrated encoder position cannot be set.");
        }

        default boolean setPositionTarget(double rotations, int slot, double feedforwardVolts, boolean enableFoc) {
            setPositionTarget(rotations);
            return false;
        }

        default boolean setVelocityTarget(
                double rotationsPerSecond,
                int slot,
                double feedforwardVolts,
                boolean enableFoc) {
            setVelocityTarget(rotationsPerSecond);
            return false;
        }

        default boolean configureSlot(MotorClosedLoopConfig config) {
            return true;
        }

        default void follow(int leaderId, boolean inverted) {
            throw new UnsupportedOperationException("Hardware following is unavailable.");
        }

        void stop();

        boolean configureOutput(boolean brake, boolean inverted);

        default boolean configureCurrentLimits(int supplyAmps, int statorAmps, int torqueAmps) {
            return true;
        }

        double positionRotations();

        double velocityRotationsPerSecond();

        /** Refreshes all cached status signals in one batched call before the getters are read. */
        default void refreshSignals() { }

        default double appliedVoltage() { return 0.0; }

        default double supplyCurrentAmps() { return 0.0; }

        default double statorCurrentAmps() { return 0.0; }

        /** Device temperature in Celsius from the last batched refresh, or empty when the controller has none. */
        default java.util.OptionalDouble temperatureCelsius() { return java.util.OptionalDouble.empty(); }

        /** Whether a fault that stops the controller from driving was active at the last batched refresh. */
        default boolean fatalFault() { return false; }
    }

    private static final class PhoenixTalonController implements TalonController {
        @Override public void close() { talon.close(); }
        private final TalonFX talon;
        private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
        private final VoltageOut voltage = new VoltageOut(0.0);
        private final NeutralOut neutral = new NeutralOut();
        private final PositionVoltage positionVoltage = new PositionVoltage(0.0);
        private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);
        private final PositionVoltage simplePositionVoltage = new PositionVoltage(0.0);
        private final VelocityVoltage simpleVelocityVoltage = new VelocityVoltage(0.0);
        private final Follower follower = new Follower(0, MotorAlignmentValue.Aligned);
        private final TalonSignals signals;
        private final PendingPositionSet pendingPosition = new PendingPositionSet();

        private PhoenixTalonController(MotorDevice device) {
            talon = new TalonFX(device.id(), new CANBus(device.canbus()));
            signals = new TalonSignals(
                    talon.getPosition(),
                    talon.getVelocity(),
                    talon.getMotorVoltage(),
                    talon.getSupplyCurrent(),
                    talon.getStatorCurrent(),
                    talon.getDeviceTemp(),
                    talon.getFault_Hardware());
        }

        @Override public boolean isConnected() { return signals.freshness.connected(); }

        @Override
        public void setPercent(double percent) {
            talon.set(percent);
        }

        @Override
        public void setVoltage(double volts) {
            talon.setVoltage(volts);
        }

        @Override
        public void setPositionTarget(double rotations) {
            talon.setControl(simplePositionVoltage.withPosition(rotations));
        }

        @Override
        public void setVelocityTarget(double rotationsPerSecond) {
            talon.setControl(simpleVelocityVoltage.withVelocity(rotationsPerSecond));
        }

        @Override
        public void setSensorPosition(double rotations) {
            requireOk(talon.setPosition(rotations), "encoder position");
            pendingPosition.set(rotations, Utils.getCurrentTimeSeconds());
        }

        @Override
        public boolean setPositionTarget(double rotations, int slot, double feedforwardVolts, boolean enableFoc) {
            StatusCode status = talon.setControl(positionVoltage
                    .withPosition(rotations)
                    .withSlot(slot)
                    .withFeedForward(feedforwardVolts)
                    .withEnableFOC(enableFoc));
            return status == StatusCode.UsingProFeatureOnUnlicensedDevice;
        }

        @Override
        public boolean setVelocityTarget(
                double rotationsPerSecond,
                int slot,
                double feedforwardVolts,
                boolean enableFoc) {
            StatusCode status = talon.setControl(velocityVoltage
                    .withVelocity(rotationsPerSecond)
                    .withSlot(slot)
                    .withFeedForward(feedforwardVolts)
                    .withEnableFOC(enableFoc));
            return status == StatusCode.UsingProFeatureOnUnlicensedDevice;
        }

        @Override
        public boolean configureSlot(MotorClosedLoopConfig config) {
            SlotConfigs slot = new SlotConfigs()
                    .withKP(config.p())
                    .withKI(config.i())
                    .withKD(config.d())
                    .withKS(config.staticFeedforward())
                    .withKV(config.velocityFeedforward())
                    .withKA(config.accelerationFeedforward())
                    .withKG(config.gravityFeedforward());
            slot.SlotNumber = config.slot();
            return talon.getConfigurator().apply(slot).isOK();
        }

        @Override
        public void follow(int leaderId, boolean inverted) {
            talon.setControl(follower
                    .withLeaderID(leaderId)
                    .withMotorAlignment(inverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
        }

        @Override
        public void stop() {
            talon.stopMotor();
        }

        @Override
        public boolean configureOutput(boolean brake, boolean inverted) {
            return talon.getConfigurator().apply(new MotorOutputConfigs()
                    .withNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast)
                    .withInverted(inverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)).isOK();
        }

        @Override
        public boolean configureCurrentLimits(int supplyAmps, int statorAmps, int torqueAmps) {
            return CtreMotorHandle.configureCurrentLimits(talon, supplyAmps, statorAmps, torqueAmps);
        }

        @Override
        public void refreshSignals() {
            signals.refresh();
        }

        @Override
        public double positionRotations() {
            return pendingPosition.resolve(signals.position.getValueAsDouble(), signals.positionTimeSeconds());
        }

        @Override
        public double velocityRotationsPerSecond() {
            return signals.velocity.getValueAsDouble();
        }

        @Override public double appliedVoltage() { return signals.voltage.getValueAsDouble(); }
        @Override public double supplyCurrentAmps() { return signals.supplyCurrent.getValueAsDouble(); }
        @Override public double statorCurrentAmps() { return signals.statorCurrent.getValueAsDouble(); }
        @Override public java.util.OptionalDouble temperatureCelsius() { return signals.temperatureCelsius(); }
        @Override public boolean fatalFault() { return signals.fatalFault(); }
    }

    private static final class PhoenixTalonFxsController implements TalonController {
        @Override public void close() { talon.close(); }
        private final TalonFXS talon;
        private final MotorArrangementValue arrangement;
        private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
        private final VoltageOut voltage = new VoltageOut(0.0);
        private final NeutralOut neutral = new NeutralOut();
        private final PositionVoltage positionVoltage = new PositionVoltage(0.0);
        private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);
        private final PositionVoltage simplePositionVoltage = new PositionVoltage(0.0);
        private final VelocityVoltage simpleVelocityVoltage = new VelocityVoltage(0.0);
        private final Follower follower = new Follower(0, MotorAlignmentValue.Aligned);
        private final TalonSignals signals;
        private final PendingPositionSet pendingPosition = new PendingPositionSet();

        private PhoenixTalonFxsController(MotorDevice device) {
            talon = new TalonFXS(device.id(), new CANBus(device.canbus()));
            arrangement = arrangement(device);
            signals = new TalonSignals(
                    talon.getPosition(),
                    talon.getVelocity(),
                    talon.getMotorVoltage(),
                    talon.getSupplyCurrent(),
                    talon.getStatorCurrent(),
                    talon.getDeviceTemp(),
                    talon.getFault_Hardware(),
                    talon.getFault_BridgeShort(),
                    talon.getFault_DriveDisabledHallSensor(),
                    talon.getFault_MotorArrangementNotSelected());
        }

        @Override public boolean isConnected() { return signals.freshness.connected(); }

        private static MotorArrangementValue arrangement(MotorDevice device) {
            if (!(device.kind().motorKind() instanceof MotorKinds motor)) {
                return MotorArrangementValue.Disabled;
            }
            return switch (motor) {
                case MINION -> MotorArrangementValue.Minion_JST;
                case NEO -> MotorArrangementValue.NEO_JST;
                case NEO_550 -> MotorArrangementValue.NEO550_JST;
                case NEO_VORTEX -> MotorArrangementValue.VORTEX_JST;
                case CIM, MINI_CIM, BAG, VEX_775_PRO, ANDYMARK_9015, ANDYMARK_RS775_125,
                        BANEBOTS_RS550, BANEBOTS_RS775 -> MotorArrangementValue.Brushed_DC;
                case FALCON_500, KRAKEN_X60, KRAKEN_X44 -> MotorArrangementValue.Disabled;
            };
        }

        @Override
        public void setPercent(double percent) {
            requireOk(talon.setControl(dutyCycle.withOutput(percent)), "percent output");
        }

        @Override
        public void setVoltage(double volts) {
            requireOk(talon.setControl(voltage.withOutput(volts)), "voltage output");
        }

        @Override
        public void setPositionTarget(double rotations) {
            talon.setControl(simplePositionVoltage.withPosition(rotations));
        }

        @Override
        public void setVelocityTarget(double rotationsPerSecond) {
            talon.setControl(simpleVelocityVoltage.withVelocity(rotationsPerSecond));
        }

        @Override
        public void setSensorPosition(double rotations) {
            requireOk(talon.setPosition(rotations), "encoder position");
            pendingPosition.set(rotations, Utils.getCurrentTimeSeconds());
        }

        @Override
        public boolean setPositionTarget(double rotations, int slot, double feedforwardVolts, boolean enableFoc) {
            StatusCode status = talon.setControl(positionVoltage
                    .withPosition(rotations)
                    .withSlot(slot)
                    .withFeedForward(feedforwardVolts)
                    .withEnableFOC(enableFoc));
            return status == StatusCode.UsingProFeatureOnUnlicensedDevice;
        }

        @Override
        public boolean setVelocityTarget(
                double rotationsPerSecond,
                int slot,
                double feedforwardVolts,
                boolean enableFoc) {
            StatusCode status = talon.setControl(velocityVoltage
                    .withVelocity(rotationsPerSecond)
                    .withSlot(slot)
                    .withFeedForward(feedforwardVolts)
                    .withEnableFOC(enableFoc));
            return status == StatusCode.UsingProFeatureOnUnlicensedDevice;
        }

        @Override
        public boolean configureSlot(MotorClosedLoopConfig config) {
            SlotConfigs slot = new SlotConfigs()
                    .withKP(config.p())
                    .withKI(config.i())
                    .withKD(config.d())
                    .withKS(config.staticFeedforward())
                    .withKV(config.velocityFeedforward())
                    .withKA(config.accelerationFeedforward())
                    .withKG(config.gravityFeedforward());
            slot.SlotNumber = config.slot();
            return talon.getConfigurator().apply(slot).isOK();
        }

        @Override
        public void follow(int leaderId, boolean inverted) {
            talon.setControl(follower
                    .withLeaderID(leaderId)
                    .withMotorAlignment(inverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
        }

        @Override
        public void stop() {
            requireOk(talon.setControl(neutral), "neutral output");
        }

        @Override
        public boolean configureOutput(boolean brake, boolean inverted) {
            boolean commutationApplied = talon.getConfigurator().apply(new CommutationConfigs()
                    .withMotorArrangement(arrangement)).isOK();
            boolean outputApplied = talon.getConfigurator().apply(new MotorOutputConfigs()
                    .withNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast)
                    .withInverted(inverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive)).isOK();
            return commutationApplied && outputApplied;
        }

        @Override
        public boolean configureCurrentLimits(int supplyAmps, int statorAmps, int torqueAmps) {
            return CtreMotorHandle.configureCurrentLimits(talon, supplyAmps, statorAmps, torqueAmps);
        }

        @Override
        public void refreshSignals() {
            signals.refresh();
        }

        @Override
        public double positionRotations() {
            return pendingPosition.resolve(signals.position.getValueAsDouble(), signals.positionTimeSeconds());
        }

        @Override
        public double velocityRotationsPerSecond() {
            return signals.velocity.getValueAsDouble();
        }

        @Override public double appliedVoltage() { return signals.voltage.getValueAsDouble(); }
        @Override public double supplyCurrentAmps() { return signals.supplyCurrent.getValueAsDouble(); }
        @Override public double statorCurrentAmps() { return signals.statorCurrent.getValueAsDouble(); }
        @Override public java.util.OptionalDouble temperatureCelsius() { return signals.temperatureCelsius(); }
        @Override public boolean fatalFault() { return signals.fatalFault(); }
    }

    private static boolean configureCurrentLimits(
            TalonFX talon,
            int supplyAmps,
            int statorAmps,
            int torqueAmps) {
        CurrentLimitsConfigs limits = currentLimits(supplyAmps, statorAmps);
        boolean currentLimitsApplied = talon.getConfigurator().apply(limits).isOK();
        if (torqueAmps <= 0) {
            return currentLimitsApplied;
        }
        return currentLimitsApplied && talon.getConfigurator().apply(new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(torqueAmps)
                .withPeakReverseTorqueCurrent(-torqueAmps)).isOK();
    }

    private static boolean configureCurrentLimits(
            TalonFXS talon,
            int supplyAmps,
            int statorAmps,
            int torqueAmps) {
        // TalonFXS has no TorqueCurrentConfigs; a torque limit is not applicable, not a failure.
        if (torqueAmps > 0) {
            DriverStation.reportWarning("CTRE TalonFXS " + talon.getDeviceID()
                    + " ignores torqueCurrentLimit(" + torqueAmps + "); use stator/supply limits.", false);
        }
        CurrentLimitsConfigs limits = currentLimits(supplyAmps, statorAmps);
        return talon.getConfigurator().apply(limits).isOK();
    }

    /** Status signals read every loop, refreshed together with one CAN/JNI call. */
    private static final class TalonSignals {
        private final StatusSignal<Angle> position;
        private final StatusSignal<AngularVelocity> velocity;
        private final StatusSignal<Voltage> voltage;
        private final StatusSignal<Current> supplyCurrent;
        private final StatusSignal<Current> statorCurrent;
        private final StatusSignal<Temperature> deviceTemp;
        /** Faults that stop the controller from driving; refreshed in the same batch as the inputs. */
        private final BaseStatusSignal[] fatalFaults;
        private final BaseStatusSignal[] all;
        private final SignalFreshness freshness;

        private TalonSignals(
                StatusSignal<Angle> position,
                StatusSignal<AngularVelocity> velocity,
                StatusSignal<Voltage> voltage,
                StatusSignal<Current> supplyCurrent,
                StatusSignal<Current> statorCurrent,
                StatusSignal<Temperature> deviceTemp,
                BaseStatusSignal... fatalFaults) {
            this.position = position;
            this.velocity = velocity;
            this.voltage = voltage;
            this.supplyCurrent = supplyCurrent;
            this.statorCurrent = statorCurrent;
            this.deviceTemp = deviceTemp;
            this.fatalFaults = fatalFaults;
            BaseStatusSignal[] inputs = {position, velocity, voltage, supplyCurrent, statorCurrent, deviceTemp};
            all = java.util.Arrays.copyOf(inputs, inputs.length + fatalFaults.length);
            System.arraycopy(fatalFaults, 0, all, inputs.length, fatalFaults.length);
            freshness = new SignalFreshness(position);
        }

        private boolean fatalFault() {
            // getValueAsDouble() avoids boxing the Boolean signal value.
            for (BaseStatusSignal fault : fatalFaults) {
                if (fault.getValueAsDouble() != 0.0) {
                    return true;
                }
            }
            return false;
        }

        private java.util.OptionalDouble temperatureCelsius() {
            // Phoenix reports DeviceTemp in degrees Celsius; a missing or invalid reading is empty, not zero.
            if (deviceTemp == null || !deviceTemp.getStatus().isOK()) {
                return java.util.OptionalDouble.empty();
            }
            double value = deviceTemp.getValueAsDouble();
            return Double.isFinite(value) ? java.util.OptionalDouble.of(value) : java.util.OptionalDouble.empty();
        }

        private void refresh() {
            BaseStatusSignal.refreshAll(all);
        }

        private double positionTimeSeconds() {
            return position.getAllTimestamps().getSystemTimestamp().getTime();
        }
    }

    private static CurrentLimitsConfigs currentLimits(int supplyAmps, int statorAmps) {
        return new CurrentLimitsConfigs()
                .withSupplyCurrentLimit(Math.max(0, supplyAmps))
                .withSupplyCurrentLimitEnable(supplyAmps > 0)
                .withStatorCurrentLimit(Math.max(0, statorAmps))
                .withStatorCurrentLimitEnable(statorAmps > 0);
    }

    private static void requireOk(StatusCode status, String operation) {
        if (!status.isOK()) {
            throw new IllegalStateException("CTRE " + operation + " failed: " + status);
        }
    }
}
