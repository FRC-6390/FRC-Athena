package ca.frc6390.athena.vendor.rev;

import ca.frc6390.athena.hardware.backend.EncoderDevice;
import ca.frc6390.athena.hardware.encoder.EncoderSpec;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import java.util.Objects;

/**
 * REV through-bore absolute encoder read through WPILib duty cycle input.
 */
public final class RevThroughBoreEncoderDevice implements EncoderDevice {
    private final EncoderSpec spec;
    private final ThroughBoreController controller;

    /**
     * Creates a through-bore device using a real WPILib duty cycle encoder.
     *
     * @param spec normalized encoder spec
     */
    public RevThroughBoreEncoderDevice(EncoderSpec spec) {
        this(spec, new WpilibThroughBoreController(spec));
    }

    RevThroughBoreEncoderDevice(EncoderSpec spec, ThroughBoreController controller) {
        this.spec = Objects.requireNonNull(spec, "spec");
        this.controller = Objects.requireNonNull(controller, "controller");
    }

    @Override
    public EncoderSpec spec() {
        return spec;
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

        private WpilibThroughBoreController(EncoderSpec spec) {
            encoder = new DutyCycleEncoder(spec.id());
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
