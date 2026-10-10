package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.Objects;
import java.util.function.Consumer;

/**
 * CTRE CANcoder handle backed by Phoenix 6.
 */
public final class CtreEncoderHandle implements EncoderHandle {
    /**
     * Discontinuity point Athena expects from the CANcoder (Phoenix factory default): absolute
     * position is reported in [-0.5, 0.5) rotations before Athena applies its software offset.
     */
    static final double ABSOLUTE_DISCONTINUITY_POINT_ROTATIONS = 0.5;

    private static final double CONFIG_TOLERANCE_ROTATIONS = 1.0e-6;

    private final EncoderDevice device;
    private final CANCoderController controller;
    private final Consumer<String> warnings;
    private boolean activated;
    private boolean inputsFresh;
    private double positionRotations;
    private double absolutePositionRotations;
    private double velocityRotationsPerSecond;

    /**
     * Creates a CTRE encoder handle using a real Phoenix 6 CANcoder.
     *
     * @param device encoder declaration
     */
    public CtreEncoderHandle(EncoderDevice device) {
        this(device, new PhoenixCANCoderController(device), message -> DriverStation.reportWarning(message, false));
    }

    CtreEncoderHandle(EncoderDevice device, CANCoderController controller) {
        this(device, controller, message -> { });
    }

    CtreEncoderHandle(EncoderDevice device, CANCoderController controller, Consumer<String> warnings) {
        this.device = Objects.requireNonNull(device, "device");
        this.controller = Objects.requireNonNull(controller, "controller");
        this.warnings = Objects.requireNonNull(warnings, "warnings");
    }

    @Override
    public EncoderDevice device() {
        return device;
    }

    /**
     * Checks (or, when opted in, writes) the CANcoder magnet-sensor configuration.
     *
     * <p>Athena applies the declared offset and inversion in software
     * ({@link EncoderDevice#positionFromRotations(double)}), so an offset, direction, or
     * discontinuity stored on the device is applied on top of it. Offsets in robot code are usually
     * measured against that stored configuration, so by default it is only read and a warning is
     * reported when it differs from the raw baseline (offset 0, counter-clockwise positive,
     * discontinuity 0.5). With {@link EncoderDevice#applyBaselineConfig()} the baseline is written
     * instead, and a write failure fails activation.</p>
     */
    @Override
    public void activate() {
        if (activated) {
            return;
        }
        if (device.baselineConfigApplied()) {
            String failure = controller.configureBaseline();
            if (failure != null) {
                throw new IllegalStateException("Failed to configure CANcoder magnet sensor (offset 0, CCW+, "
                        + "discontinuity " + ABSOLUTE_DISCONTINUITY_POINT_ROTATIONS + ") for "
                        + device.defaultName() + ": " + failure);
            }
        } else {
            warnIfStoredConfigDiffers();
        }
        activated = true;
    }

    private void warnIfStoredConfigDiffers() {
        MagnetConfig stored;
        try {
            stored = controller.readMagnetConfig();
        } catch (RuntimeException exception) {
            stored = null;
        }
        if (stored == null) {
            warnings.accept("Could not read CANcoder magnet sensor config for " + device.defaultName()
                    + "; Athena assumes offset 0, CCW+, discontinuity " + ABSOLUTE_DISCONTINUITY_POINT_ROTATIONS
                    + ".");
            return;
        }
        if (Math.abs(stored.magnetOffsetRotations()) > CONFIG_TOLERANCE_ROTATIONS
                || !stored.counterClockwisePositive()
                || Math.abs(stored.discontinuityPointRotations() - ABSOLUTE_DISCONTINUITY_POINT_ROTATIONS)
                        > CONFIG_TOLERANCE_ROTATIONS) {
            warnings.accept("CANcoder " + device.defaultName() + " has stored magnet config (offset "
                    + stored.magnetOffsetRotations() + " rot, "
                    + (stored.counterClockwisePositive() ? "CCW+" : "CW+") + ", discontinuity "
                    + stored.discontinuityPointRotations() + " rot) that differs from Athena's raw baseline "
                    + "(offset 0, CCW+, discontinuity " + ABSOLUTE_DISCONTINUITY_POINT_ROTATIONS + "). It is applied "
                    + "in addition to the offset/inversion declared in code. Keep it if your code offsets were "
                    + "measured with it, or use EncoderDevice.applyBaselineConfig() and re-measure.");
        }
    }

    /** Magnet-sensor configuration stored on a CANcoder. */
    record MagnetConfig(
            double magnetOffsetRotations,
            boolean counterClockwisePositive,
            double discontinuityPointRotations) {
    }

    @Override
    public double positionRotations() {
        ensureInputsFresh();
        return positionRotations;
    }

    @Override
    public double absolutePositionRotations() {
        ensureInputsFresh();
        return absolutePositionRotations;
    }

    @Override
    public double velocityRotationsPerSecond() {
        ensureInputsFresh();
        return velocityRotationsPerSecond;
    }

    @Override
    public void setPositionRotations(double rotations) {
        double safeRotations = finiteOrZero(rotations);
        if (!controller.setPositionRotations(safeRotations)) {
            throw new IllegalStateException("Failed to set position for " + device.defaultName());
        }
        positionRotations = safeRotations;
        inputsFresh = true;
    }

    @Override
    public boolean supportsPositionSetting() {
        return true;
    }

    @Override
    public void refreshInputs() {
        if (!controller.isConnected()) {
            throw new IllegalStateException("CTRE encoder is disconnected: " + device.defaultName());
        }
        controller.refreshSignals();
        positionRotations = controller.positionRotations();
        absolutePositionRotations = controller.absolutePositionRotations();
        velocityRotationsPerSecond = controller.velocityRotationsPerSecond();
        inputsFresh = true;
    }

    private void ensureInputsFresh() {
        if (!inputsFresh) {
            refreshInputs();
        }
    }

    interface CANCoderController {
        default boolean isConnected() { return true; }

        /**
         * Applies the baseline magnet-sensor configuration.
         *
         * @return null on success, otherwise a failure description
         */
        default String configureBaseline() { return null; }

        /**
         * Reads the stored magnet-sensor configuration.
         *
         * @return stored configuration, or null when it could not be read
         */
        default MagnetConfig readMagnetConfig() {
            return new MagnetConfig(0.0, true, ABSOLUTE_DISCONTINUITY_POINT_ROTATIONS);
        }

        /** Refreshes all cached status signals in one batched call. */
        default void refreshSignals() { }

        double positionRotations();

        double absolutePositionRotations();

        double velocityRotationsPerSecond();

        boolean setPositionRotations(double rotations);
    }

    private static final class PhoenixCANCoderController implements CANCoderController {
        private static final int CONFIG_ATTEMPTS = 3;
        private final CANcoder encoder;
        private final StatusSignal<Angle> position;
        private final StatusSignal<Angle> absolutePosition;
        private final StatusSignal<AngularVelocity> velocity;
        private final BaseStatusSignal[] signals;
        private final PendingPositionSet pendingPosition = new PendingPositionSet();

        private PhoenixCANCoderController(EncoderDevice device) {
            encoder = new CANcoder(device.id(), new CANBus(device.canbus()));
            position = encoder.getPosition();
            absolutePosition = encoder.getAbsolutePosition();
            velocity = encoder.getVelocity();
            signals = new BaseStatusSignal[] {position, absolutePosition, velocity};
        }

        @Override public boolean isConnected() { return encoder.isConnected(); }

        @Override
        public String configureBaseline() {
            MagnetSensorConfigs baseline = new MagnetSensorConfigs()
                    .withMagnetOffset(0.0)
                    .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
                    .withAbsoluteSensorDiscontinuityPoint(ABSOLUTE_DISCONTINUITY_POINT_ROTATIONS);
            StatusCode status = StatusCode.OK;
            for (int attempt = 0; attempt < CONFIG_ATTEMPTS; attempt++) {
                status = encoder.getConfigurator().apply(baseline);
                if (status.isOK()) {
                    return null;
                }
            }
            return status.toString();
        }

        @Override
        public MagnetConfig readMagnetConfig() {
            MagnetSensorConfigs stored = new MagnetSensorConfigs();
            if (!encoder.getConfigurator().refresh(stored).isOK()) {
                return null;
            }
            return new MagnetConfig(
                    stored.MagnetOffset,
                    stored.SensorDirection == SensorDirectionValue.CounterClockwise_Positive,
                    stored.AbsoluteSensorDiscontinuityPoint);
        }

        @Override
        public void refreshSignals() {
            BaseStatusSignal.refreshAll(signals);
        }

        @Override
        public double positionRotations() {
            return pendingPosition.resolve(
                    position.getValueAsDouble(),
                    position.getAllTimestamps().getSystemTimestamp().getTime());
        }

        @Override
        public double absolutePositionRotations() {
            return absolutePosition.getValueAsDouble();
        }

        @Override
        public double velocityRotationsPerSecond() {
            return velocity.getValueAsDouble();
        }

        @Override
        public boolean setPositionRotations(double rotations) {
            boolean ok = encoder.setPosition(rotations).isOK();
            if (ok) {
                pendingPosition.set(rotations, Utils.getCurrentTimeSeconds());
            }
            return ok;
        }
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
