package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.Units;
import java.util.Objects;

/**
 * CTRE CANcoder handle backed by Phoenix 6.
 */
public final class CtreEncoderHandle implements EncoderHandle {
    private final EncoderDevice device;
    private final CANCoderController controller;
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
        this(device, new PhoenixCANCoderController(device));
    }

    CtreEncoderHandle(EncoderDevice device, CANCoderController controller) {
        this.device = Objects.requireNonNull(device, "device");
        this.controller = Objects.requireNonNull(controller, "controller");
    }

    @Override
    public EncoderDevice device() {
        return device;
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

        double positionRotations();

        double absolutePositionRotations();

        double velocityRotationsPerSecond();

        boolean setPositionRotations(double rotations);
    }

    private static final class PhoenixCANCoderController implements CANCoderController {
        private final CANcoder encoder;

        private PhoenixCANCoderController(EncoderDevice device) {
            encoder = new CANcoder(device.id(), new CANBus(device.canbus()));
        }

        @Override public boolean isConnected() { return encoder.isConnected(); }

        @Override
        public double positionRotations() {
            return encoder.getPosition().refresh().getValue().in(Units.Rotations);
        }

        @Override
        public double absolutePositionRotations() {
            return encoder.getAbsolutePosition().refresh().getValue().in(Units.Rotations);
        }

        @Override
        public double velocityRotationsPerSecond() {
            return encoder.getVelocity().refresh().getValue().in(Units.RotationsPerSecond);
        }

        @Override
        public boolean setPositionRotations(double rotations) {
            return encoder.setPosition(rotations).isOK();
        }
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }
}
