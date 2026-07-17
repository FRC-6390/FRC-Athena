package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.hardware.backend.FocPolicy;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopConfig;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import ca.frc6390.athena.api.hardware.MotorControllerKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
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
import edu.wpi.first.units.Units;
import java.util.Objects;

/**
 * CTRE TalonFX-family motor handle backed by Phoenix 6.
 */
public final class CtreMotorHandle implements MotorHandle {
    private final MotorDevice device;
    private final CtreMotorOptions options;
    private final TalonController controller;
    private boolean activated;
    private boolean inputsFresh;
    private double positionRotations;
    private double velocityRotationsPerSecond;
    private double appliedVoltage;
    private double supplyCurrentAmps;
    private double statorCurrentAmps;
    private MotorClosedLoopConfig appliedClosedLoopConfig;
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
    public void applyRuntimeConfiguration(MotorRuntimeConfig configuration) {
        Objects.requireNonNull(configuration, "configuration");
        if (!controller.configureOutput(
                configuration.neutralMode() == MotorNeutralMode.BRAKE,
                device.follower() == null && configuration.inverted())) {
            throw new IllegalStateException("Failed to configure output for " + device.defaultName());
        }
        if (!controller.configureCurrentLimits(
                configuration.supplyCurrentLimitAmps(),
                configuration.statorCurrentLimitAmps(),
                options.torqueCurrentLimitAmps())) {
            throw new IllegalStateException("Failed to configure current limits for " + device.defaultName());
        }
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
        if (!activated) {
            if (!controller.configureOutput(
                    device.neutralMode() == MotorNeutralMode.BRAKE,
                    device.follower() == null && device.isInverted())) {
                throw new IllegalStateException("Failed to configure output for " + device.defaultName());
            }
            int supplyLimit = options.supplyCurrentLimitAmps() > 0
                    ? options.supplyCurrentLimitAmps()
                    : device.supplyCurrentLimitAmps() > 0
                            ? device.supplyCurrentLimitAmps()
                            : device.currentLimitAmps();
            int statorLimit = options.statorCurrentLimitAmps() > 0
                    ? options.statorCurrentLimitAmps()
                    : device.statorCurrentLimitAmps();
            if (!controller.configureCurrentLimits(
                    supplyLimit,
                    statorLimit,
                    options.torqueCurrentLimitAmps())) {
                throw new IllegalStateException("Failed to configure current limits for " + device.defaultName());
            }
            activated = true;
        }
    }

    @Override
    public void refreshInputs() {
        if (!controller.isConnected()) {
            throw new IllegalStateException("CTRE motor is disconnected: " + device.defaultName());
        }
        positionRotations = controller.positionRotations();
        velocityRotationsPerSecond = controller.velocityRotationsPerSecond();
        appliedVoltage = controller.appliedVoltage();
        supplyCurrentAmps = controller.supplyCurrentAmps();
        statorCurrentAmps = controller.statorCurrentAmps();
        inputsFresh = true;
    }

    @Override
    public void follow(MotorHandle leader, boolean inverted) {
        if (!(leader instanceof CtreMotorHandle ctreLeader)) {
            throw new IllegalArgumentException("CTRE motor " + device.defaultName()
                    + " can only follow another CTRE motor.");
        }
        controller.follow(ctreLeader.device.id(), inverted);
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
        MotorClosedLoopRequest safeRequest = safeRequest(request);
        applyClosedLoopConfig(safeRequest.config());
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
        controller.setVelocityTarget(finiteOrZero(rotationsPerSecond));
    }

    @Override
    public void setVelocityTargetRotationsPerSecond(
            double rotationsPerSecond,
            MotorClosedLoopRequest request) {
        MotorClosedLoopRequest safeRequest = safeRequest(request);
        applyClosedLoopConfig(safeRequest.config());
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

    @Override
    public void setIntegratedPositionRotations(double rotations) {
        double safeRotations = finiteOrZero(rotations);
        controller.setSensorPosition(safeRotations);
        positionRotations = safeRotations;
        inputsFresh = true;
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

    private void applyClosedLoopConfig(MotorClosedLoopConfig config) {
        if (config.equals(appliedClosedLoopConfig)) {
            return;
        }
        if (!controller.configureSlot(config)) {
            throw new IllegalStateException("Failed to configure closed-loop slot for " + device.defaultName());
        }
        appliedClosedLoopConfig = config;
    }

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

        default double appliedVoltage() { return 0.0; }

        default double supplyCurrentAmps() { return 0.0; }

        default double statorCurrentAmps() { return 0.0; }
    }

    private static final class PhoenixTalonController implements TalonController {
        private final TalonFX talon;
        private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
        private final VoltageOut voltage = new VoltageOut(0.0);
        private final NeutralOut neutral = new NeutralOut();
        private final PositionVoltage positionVoltage = new PositionVoltage(0.0);
        private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

        private PhoenixTalonController(MotorDevice device) {
            talon = new TalonFX(device.id(), new CANBus(device.canbus()));
        }

        @Override public boolean isConnected() { return talon.isConnected(); }

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
            talon.setControl(new PositionVoltage(rotations));
        }

        @Override
        public void setVelocityTarget(double rotationsPerSecond) {
            talon.setControl(new VelocityVoltage(rotationsPerSecond));
        }

        @Override
        public void setSensorPosition(double rotations) {
            talon.setPosition(rotations);
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
            talon.setControl(new Follower(
                    leaderId,
                    inverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
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
        public double positionRotations() {
            return talon.getPosition().refresh().getValue().in(Units.Rotations);
        }

        @Override
        public double velocityRotationsPerSecond() {
            return talon.getVelocity().refresh().getValue().in(Units.RotationsPerSecond);
        }

        @Override public double appliedVoltage() { return talon.getMotorVoltage().refresh().getValue().in(Units.Volts); }
        @Override public double supplyCurrentAmps() { return talon.getSupplyCurrent().refresh().getValue().in(Units.Amps); }
        @Override public double statorCurrentAmps() { return talon.getStatorCurrent().refresh().getValue().in(Units.Amps); }
    }

    private static final class PhoenixTalonFxsController implements TalonController {
        private final TalonFXS talon;
        private final MotorArrangementValue arrangement;
        private final DutyCycleOut dutyCycle = new DutyCycleOut(0.0);
        private final VoltageOut voltage = new VoltageOut(0.0);
        private final NeutralOut neutral = new NeutralOut();
        private final PositionVoltage positionVoltage = new PositionVoltage(0.0);
        private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

        private PhoenixTalonFxsController(MotorDevice device) {
            talon = new TalonFXS(device.id(), new CANBus(device.canbus()));
            arrangement = arrangement(device);
        }

        @Override public boolean isConnected() { return talon.isConnected(); }

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
            talon.setControl(new PositionVoltage(rotations));
        }

        @Override
        public void setVelocityTarget(double rotationsPerSecond) {
            talon.setControl(new VelocityVoltage(rotationsPerSecond));
        }

        @Override
        public void setSensorPosition(double rotations) {
            talon.setPosition(rotations);
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
            talon.setControl(new Follower(
                    leaderId,
                    inverted ? MotorAlignmentValue.Opposed : MotorAlignmentValue.Aligned));
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
        public double positionRotations() {
            return talon.getPosition().refresh().getValue().in(Units.Rotations);
        }

        @Override
        public double velocityRotationsPerSecond() {
            return talon.getVelocity().refresh().getValue().in(Units.RotationsPerSecond);
        }

        @Override public double appliedVoltage() { return talon.getMotorVoltage().refresh().getValue().in(Units.Volts); }
        @Override public double supplyCurrentAmps() { return talon.getSupplyCurrent().refresh().getValue().in(Units.Amps); }
        @Override public double statorCurrentAmps() { return talon.getStatorCurrent().refresh().getValue().in(Units.Amps); }
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
        CurrentLimitsConfigs limits = currentLimits(supplyAmps, statorAmps);
        boolean currentLimitsApplied = talon.getConfigurator().apply(limits).isOK();
        return currentLimitsApplied && torqueAmps <= 0;
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
