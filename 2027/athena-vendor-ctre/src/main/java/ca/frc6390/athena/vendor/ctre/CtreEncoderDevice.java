package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.hardware.backend.EncoderDevice;
import ca.frc6390.athena.hardware.encoder.EncoderSpec;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.units.Units;
import java.util.Objects;

/**
 * CTRE CANcoder device backed by Phoenix 6.
 */
public final class CtreEncoderDevice implements EncoderDevice {
    private final EncoderSpec spec;
    private final CANCoderController controller;

    /**
     * Creates a CTRE encoder device using a real Phoenix 6 CANcoder.
     *
     * @param spec normalized encoder spec
     */
    public CtreEncoderDevice(EncoderSpec spec) {
        this(spec, new PhoenixCANCoderController(spec));
    }

    CtreEncoderDevice(EncoderSpec spec, CANCoderController controller) {
        this.spec = Objects.requireNonNull(spec, "spec");
        this.controller = Objects.requireNonNull(controller, "controller");
    }

    @Override
    public EncoderSpec spec() {
        return spec;
    }

    @Override
    public double positionRotations() {
        return controller.positionRotations();
    }

    @Override
    public double absolutePositionRotations() {
        return controller.absolutePositionRotations();
    }

    @Override
    public double velocityRotationsPerSecond() {
        return controller.velocityRotationsPerSecond();
    }

    interface CANCoderController {
        double positionRotations();

        double absolutePositionRotations();

        double velocityRotationsPerSecond();
    }

    private static final class PhoenixCANCoderController implements CANCoderController {
        private final CANcoder encoder;

        private PhoenixCANCoderController(EncoderSpec spec) {
            encoder = new CANcoder(spec.id(), new CANBus(spec.canbus()));
        }

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
    }
}
