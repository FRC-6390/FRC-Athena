package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.hardware.backend.FocPolicy;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopConfig;
import ca.frc6390.athena.hardware.backend.MotorClosedLoopRequest;
import ca.frc6390.athena.hardware.backend.MotorControlCapabilities;
import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import ca.frc6390.athena.api.hardware.MotorControllerKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.CommutationConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
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
            controller.configureOutput(
                    device.neutralMode() == MotorNeutralMode.BRAKE,
                    device.follower() == null && device.isInverted());
            activated = true;
        }
    }

    @Override
    public void refreshInputs() {
        positionRotations = controller.positionRotations();
        velocityRotationsPerSecond = controller.velocityRotationsPerSecond();
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
        controller.configureSlot(config);
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

        default void configureSlot(MotorClosedLoopConfig config) {
        }

        default void follow(int leaderId, boolean inverted) {
            throw new UnsupportedOperationException("Hardware following is unavailable.");
        }

        void stop();

        void configureOutput(boolean brake, boolean inverted);

        double positionRotations();

        double velocityRotationsPerSecond();
    }

    private static final class PhoenixTalonController implements TalonController {
        private final TalonFX talon;
        private final PositionVoltage positionVoltage = new PositionVoltage(0.0);
        private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

        private PhoenixTalonController(MotorDevice device) {
            talon = new TalonFX(device.id(), new CANBus(device.canbus()));
        }

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
        public void configureSlot(MotorClosedLoopConfig config) {
            SlotConfigs slot = new SlotConfigs()
                    .withKP(config.p())
                    .withKI(config.i())
                    .withKD(config.d())
                    .withKS(config.staticFeedforward())
                    .withKV(config.velocityFeedforward())
                    .withKG(config.gravityFeedforward());
            slot.SlotNumber = config.slot();
            talon.getConfigurator().apply(slot);
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
        public void configureOutput(boolean brake, boolean inverted) {
            talon.getConfigurator().apply(new MotorOutputConfigs()
                    .withNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast)
                    .withInverted(inverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive));
        }

        @Override
        public double positionRotations() {
            return talon.getPosition().refresh().getValue().in(Units.Rotations);
        }

        @Override
        public double velocityRotationsPerSecond() {
            return talon.getVelocity().refresh().getValue().in(Units.RotationsPerSecond);
        }
    }

    private static final class PhoenixTalonFxsController implements TalonController {
        private final TalonFXS talon;
        private final PositionVoltage positionVoltage = new PositionVoltage(0.0);
        private final VelocityVoltage velocityVoltage = new VelocityVoltage(0.0);

        private PhoenixTalonFxsController(MotorDevice device) {
            talon = new TalonFXS(device.id(), new CANBus(device.canbus()));
            talon.getConfigurator().apply(new CommutationConfigs()
                    .withMotorArrangement(arrangement(device)));
        }

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
        public void configureSlot(MotorClosedLoopConfig config) {
            SlotConfigs slot = new SlotConfigs()
                    .withKP(config.p())
                    .withKI(config.i())
                    .withKD(config.d())
                    .withKS(config.staticFeedforward())
                    .withKV(config.velocityFeedforward())
                    .withKG(config.gravityFeedforward());
            slot.SlotNumber = config.slot();
            talon.getConfigurator().apply(slot);
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
        public void configureOutput(boolean brake, boolean inverted) {
            talon.getConfigurator().apply(new MotorOutputConfigs()
                    .withNeutralMode(brake ? NeutralModeValue.Brake : NeutralModeValue.Coast)
                    .withInverted(inverted
                            ? InvertedValue.Clockwise_Positive
                            : InvertedValue.CounterClockwise_Positive));
        }

        @Override
        public double positionRotations() {
            return talon.getPosition().refresh().getValue().in(Units.Rotations);
        }

        @Override
        public double velocityRotationsPerSecond() {
            return talon.getVelocity().refresh().getValue().in(Units.RotationsPerSecond);
        }
    }
}
