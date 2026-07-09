package ca.frc6390.athena.vendor.rev;

import ca.frc6390.athena.hardware.backend.EncoderHandle;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.Objects;

/**
 * REV through-bore absolute encoder handle read through WPILib duty cycle input.
 */
public final class RevThroughBoreEncoderHandle implements EncoderHandle {
    private final EncoderDevice device;
    private final ThroughBoreController controller;

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
    public double absolutePositionRotations() {
        return controller.absolutePositionRotations();
    }

    @Override
    public double positionRotations() {
        return absolutePositionRotations();
    }

    @Override
    public double velocityRotationsPerSecond() {
        return controller.velocityRotationsPerSecond();
    }

    interface ThroughBoreController {
        double absolutePositionRotations();

        double velocityRotationsPerSecond();
    }

    private static final class WpilibThroughBoreController implements ThroughBoreController {
        private final DutyCycleEncoder encoder;

        private WpilibThroughBoreController(EncoderDevice device) {
            encoder = new DutyCycleEncoder(device.dioChannel());
        }

        @Override
        public double absolutePositionRotations() {
            return encoder.get();
        }

        @Override
        public double velocityRotationsPerSecond() {
            throw new UnsupportedOperationException("REV through-bore velocity is not available from duty cycle input.");
        }
    }
}
