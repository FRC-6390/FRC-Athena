package ca.frc6390.athena.vendor.rev;

import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.HardwareAddress;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.Objects;

/**
 * REV through-bore absolute encoder handle read through WPILib duty cycle input.
 */
public final class RevThroughBoreEncoderHandle implements EncoderHandle, AutoCloseable {
    private final EncoderDevice device;
    private final ThroughBoreController controller;
    private volatile double absolutePositionRotations = Double.NaN;

    /**
     * Creates a through-bore handle using a real WPILib duty cycle encoder.
     *
     * @param device encoder declaration
     */
    public RevThroughBoreEncoderHandle(EncoderDevice device) {
        this(device, new WpilibThroughBoreController(device));
    }

    RevThroughBoreEncoderHandle(EncoderDevice device, ThroughBoreController controller) {
        this.device = Objects.requireNonNull(device, "device");
        this.controller = Objects.requireNonNull(controller, "controller");
    }

    @Override
    public EncoderDevice device() {
        return device;
    }

    @Override
    public void refreshInputs() {
        if (!controller.isConnected()) {
            throw new IllegalStateException("REV through-bore encoder " + device.defaultName()
                    + " is disconnected or not producing a duty-cycle signal.");
        }
        absolutePositionRotations = controller.absolutePositionRotations();
    }

    @Override
    public double absolutePositionRotations() {
        return absolutePositionRotations;
    }

    @Override
    public double positionRotations() {
        return absolutePositionRotations();
    }

    @Override
    public double velocityRotationsPerSecond() {
        throw new UnsupportedOperationException("REV through-bore velocity is not available from duty cycle input.");
    }

    @Override
    public void close() {
        controller.close();
    }

    interface ThroughBoreController {
        double absolutePositionRotations();

        double velocityRotationsPerSecond();

        default boolean isConnected() { return true; }

        default void close() {}
    }

    private static final class WpilibThroughBoreController implements ThroughBoreController {
        private final DutyCycleEncoder encoder;

        private WpilibThroughBoreController(EncoderDevice device) {
            if (!(device.connection() instanceof HardwareAddress.Dio dio)) {
                throw new IllegalArgumentException(
                        "REV through-bore absolute input requires a single DIO connection.");
            }
            encoder = new DutyCycleEncoder(dio.channel());
            encoder.setConnectedFrequencyThreshold(100);
            if (device.kind() == EncoderKinds.REV_THROUGH_BORE_V2
                    || device.kind().key().equals("rev:through-bore-v2")) {
                encoder.setDutyCycleRange(3.884 / 1000.0, 998.06 / 1000.0);
            } else {
                encoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
            }
        }

        @Override
        public double absolutePositionRotations() {
            return encoder.get();
        }

        @Override
        public double velocityRotationsPerSecond() {
            throw new UnsupportedOperationException("REV through-bore velocity is not available from duty cycle input.");
        }

        @Override
        public boolean isConnected() {
            return encoder.isConnected();
        }

        @Override
        public void close() {
            encoder.close();
        }
    }
}
