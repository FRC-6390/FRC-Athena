package ca.frc6390.athena.vendor.ctre;

import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.ImuDevice;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.units.Units;
import java.util.Objects;

/**
 * CTRE Pigeon2 IMU handle backed by Phoenix 6.
 */
public final class CtrePigeon2Handle implements ImuHandle, AutoCloseable {
    private final ImuDevice device;
    private final Pigeon2Controller controller;
    private boolean inputsFresh;
    private double yawDegrees;
    private double pitchDegrees;
    private double rollDegrees;

    /**
     * Creates a CTRE Pigeon2 handle using a real Phoenix 6 Pigeon2.
     *
     * @param device IMU declaration
     */
    public CtrePigeon2Handle(ImuDevice device) {
        this(device, new PhoenixPigeon2Controller(device));
    }

    CtrePigeon2Handle(ImuDevice device, Pigeon2Controller controller) {
        this.device = Objects.requireNonNull(device, "device");
        this.controller = Objects.requireNonNull(controller, "controller");
    }

    @Override
    public ImuDevice device() {
        return device;
    }

    @Override
    public double yawDegrees() {
        ensureInputsFresh();
        return yawDegrees;
    }

    @Override
    public void refreshInputs() {
        yawDegrees = controller.yawDegrees();
        pitchDegrees = controller.pitchDegrees();
        rollDegrees = controller.rollDegrees();
        inputsFresh = true;
    }

    /**
     * Returns pitch in degrees.
     *
     * @return pitch degrees
     */
    public double pitchDegrees() {
        ensureInputsFresh();
        return pitchDegrees;
    }

    /**
     * Returns roll in degrees.
     *
     * @return roll degrees
     */
    public double rollDegrees() {
        ensureInputsFresh();
        return rollDegrees;
    }

    @Override
    public void zeroYaw() {
        controller.setYawDegrees(0.0);
        inputsFresh = false;
    }

    @Override
    public void reset() {
        controller.reset();
        inputsFresh = false;
    }

    @Override
    public void close() {
        controller.close();
    }

    private void ensureInputsFresh() {
        if (!inputsFresh) {
            refreshInputs();
        }
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

        private PhoenixPigeon2Controller(ImuDevice device) {
            pigeon = new Pigeon2(device.id(), new CANBus(device.canbus()));
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
