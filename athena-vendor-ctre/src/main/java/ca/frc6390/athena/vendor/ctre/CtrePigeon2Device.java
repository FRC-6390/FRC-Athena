package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.hardware.backend.ImuDevice;
import ca.frc6390.athena.hardware.imu.ImuSpec;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.Units;
import java.util.Objects;

/**
 * CTRE Pigeon2 IMU device backed by Phoenix 6.
 */
public final class CtrePigeon2Device implements ImuDevice, AutoCloseable {
    private final ImuSpec spec;
    private final Pigeon2Controller controller;

    /**
     * Creates a CTRE Pigeon2 device using a real Phoenix 6 Pigeon2.
     *
     * @param spec normalized IMU spec
     */
    public CtrePigeon2Device(ImuSpec spec) {
        this(spec, new PhoenixPigeon2Controller(spec));
    }

    CtrePigeon2Device(ImuSpec spec, Pigeon2Controller controller) {
        this.spec = Objects.requireNonNull(spec, "spec");
        this.controller = Objects.requireNonNull(controller, "controller");
    }

    @Override
    public ImuSpec spec() {
        return spec;
    }

    @Override
    public double yawDegrees() {
        return controller.yawDegrees();
    }

    /**
     * Returns pitch in degrees.
     *
     * @return pitch degrees
     */
    public double pitchDegrees() {
        return controller.pitchDegrees();
    }

    /**
     * Returns roll in degrees.
     *
     * @return roll degrees
     */
    public double rollDegrees() {
        return controller.rollDegrees();
    }

    /**
     * Sets the current yaw reading to zero.
     */
    public void zeroYaw() {
        controller.setYawDegrees(0.0);
    }

    /**
     * Resets the underlying Pigeon2.
     */
    public void reset() {
        controller.reset();
    }

    @Override
    public void close() {
        controller.close();
    }

    interface Pigeon2Controller extends AutoCloseable {
        double yawDegrees();

        double pitchDegrees();

        double rollDegrees();

        void setYawDegrees(double yawDegrees);

        void reset();

        @Override
        void close();
    }

    private static final class PhoenixPigeon2Controller implements Pigeon2Controller {
        private final Pigeon2 pigeon;

        private PhoenixPigeon2Controller(ImuSpec spec) {
            pigeon = new Pigeon2(spec.id(), new CANBus(spec.canbus()));
        }

        @Override
        public double yawDegrees() {
            return pigeon.getYaw().refresh().getValue().in(Units.Degrees);
        }

        @Override
        public double pitchDegrees() {
            return pigeon.getPitch().refresh().getValue().in(Units.Degrees);
        }

        @Override
        public double rollDegrees() {
            return pigeon.getRoll().refresh().getValue().in(Units.Degrees);
        }

        @Override
        public void setYawDegrees(double yawDegrees) {
            pigeon.setYaw(yawDegrees);
        }

        @Override
        public void reset() {
            pigeon.reset();
        }

        @Override
        public void close() {
            pigeon.close();
        }
    }
}
