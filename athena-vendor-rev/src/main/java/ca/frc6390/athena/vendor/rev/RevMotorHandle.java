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
import java.util.Objects;

/**
 * REV Spark motor handle backed by REVLib.
 */
public final class RevMotorHandle implements MotorHandle {
    private final MotorDevice device;
    private final RevMotorOptions options;
    private final SparkController controller;
    private boolean activated;
    private boolean inputsFresh;
    private double positionRotations;
    private double velocityRotationsPerSecond;
    private double absolutePositionRotations;
    private double absoluteVelocityRotationsPerSecond;
    private double appliedVoltage;
    private double outputCurrentAmps;

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
            if (device.supplyCurrentLimitAmps() > 0 || device.statorCurrentLimitAmps() > 0) {
                throw new IllegalStateException("REV motor " + device.defaultName()
                        + " does not support explicit supplyCurrentLimit or statorCurrentLimit settings. "
                        + "Use currentLimit(...) for the REV smart current limit.");
            }
            if (!controller.configure(device, options)) {
                throw new IllegalStateException("Failed to configure " + device.defaultName());
            }
            activated = true;
        }
    }

    @Override
    public void refreshInputs() {
        positionRotations = controller.positionRotations();
        velocityRotationsPerSecond = controller.velocityRotationsPerSecond();
        absolutePositionRotations = controller.absolutePositionRotations();
        absoluteVelocityRotationsPerSecond = controller.absoluteVelocityRotationsPerSecond();
        appliedVoltage = controller.appliedVoltage();
        outputCurrentAmps = controller.outputCurrentAmps();
        inputsFresh = true;
    }

    @Override
    public void follow(MotorHandle leader, boolean inverted) {
        if (!(leader instanceof RevMotorHandle revLeader)) {
            throw new IllegalArgumentException("REV motor " + device.defaultName()
                    + " can only follow another REV motor.");
        }
        if (!controller.follow(revLeader.device.id(), inverted)) {
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
        if (!controller.setSensorPosition(safeRotations)) {
            throw new IllegalStateException("Failed to set position for " + device.defaultName());
        }
        positionRotations = safeRotations;
        inputsFresh = true;
    }

    @Override
    public boolean supportsIntegratedPositionSetting() {
        return true;
    }

    @Override
    public double absolutePositionRotations() {
        ensureInputsFresh();
        return absolutePositionRotations;
    }

    @Override
    public double absoluteVelocityRotationsPerSecond() {
        ensureInputsFresh();
        return absoluteVelocityRotationsPerSecond;
    }

    @Override public double appliedVoltage() { ensureInputsFresh(); return appliedVoltage; }
    @Override public double supplyCurrentAmps() { ensureInputsFresh(); return outputCurrentAmps; }
    @Override public double statorCurrentAmps() { ensureInputsFresh(); return outputCurrentAmps; }

    private void ensureInputsFresh() {
        if (!inputsFresh) {
            refreshInputs();
        }
    }

    private UnsupportedOperationException unsupportedVoltageClosedLoop() {
        return new UnsupportedOperationException(
                "REV closed-loop gains use duty-cycle semantics; Athena voltage-gain requests must run in software for "
                        + device.defaultName());
    }

    private static SparkController createController(MotorDevice device) {
        boolean brushless = device.kind().technology() == MotorTechnology.BRUSHLESS
                || device.kind().technology() == MotorTechnology.UNKNOWN
                        && device.kind().key().endsWith("brushless");
        MotorType motorType = brushless ? MotorType.kBrushless : MotorType.kBrushed;
        boolean flex = device.kind().controllerKind() == MotorControllerKinds.SPARK_FLEX
                || device.kind().key().startsWith("rev:spark-flex");
        SparkBase spark = flex
                        ? new SparkFlex(device.id(), motorType)
                        : new SparkMax(device.id(), motorType);
        return new RevSparkController(spark, flex);
    }

    private static double clamp(double value) {
        double finite = finiteOrZero(value);
        return Math.max(-1.0, Math.min(1.0, finite));
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
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

        default boolean follow(int leaderId, boolean inverted) {
            throw new UnsupportedOperationException("Hardware following is unavailable.");
        }

        void stop();

        double positionRotations();

        double velocityRotationsPerSecond();

        double absolutePositionRotations();

        double absoluteVelocityRotationsPerSecond();

        default double appliedVoltage() { return 0.0; }

        default double outputCurrentAmps() { return 0.0; }
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
            SparkBaseConfig config = flex ? new SparkFlexConfig() : new SparkMaxConfig();
            config.idleMode(device.neutralMode() == MotorNeutralMode.BRAKE ? IdleMode.kBrake : IdleMode.kCoast);
            config.inverted(device.isInverted());
            int currentLimit = options.smartCurrentLimitAmps() > 0
                    ? options.smartCurrentLimitAmps()
                    : device.currentLimitAmps();
            if (currentLimit > 0) {
                config.smartCurrentLimit(currentLimit);
            }
            if (options.openLoopRampSeconds() > 0.0) {
                config.openLoopRampRate(options.openLoopRampSeconds());
            }
            if (options.closedLoopRampSeconds() > 0.0) {
                config.closedLoopRampRate(options.closedLoopRampSeconds());
            }
            return spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters)
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
        public boolean follow(int leaderId, boolean inverted) {
            SparkBaseConfig config = flex ? new SparkFlexConfig() : new SparkMaxConfig();
            config.follow(leaderId, inverted);
            return spark.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters)
                    == REVLibError.kOk;
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
