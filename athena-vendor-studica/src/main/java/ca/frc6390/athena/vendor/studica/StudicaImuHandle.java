package ca.frc6390.athena.vendor.studica;

import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.ImuDevice;
import com.studica.frc.AHRS;
import java.util.Objects;

/**
 * Studica/NavX IMU handle backed by the Studica {@link AHRS} library.
 */
public final class StudicaImuHandle implements ImuHandle, AutoCloseable {
    private final ImuDevice device;
    private final NavxController controller;

    /**
     * Creates a Studica/NavX IMU handle using a real Studica AHRS.
     *
     * @param device IMU declaration
     */
    public StudicaImuHandle(ImuDevice device) {
        this(device, new AhrsNavxController(device));
    }

    StudicaImuHandle(ImuDevice device, NavxController controller) {
        this.device = Objects.requireNonNull(device, "device");
        this.controller = Objects.requireNonNull(controller, "controller");
    }

    @Override
    public ImuDevice device() {
        return device;
    }

    @Override
    public double yawDegrees() {
        return controller.yawDegrees();
    }

    @Override
    public void activate() {
        controller.activate();
    }

    @Override
    public double angleDegrees() {
        return controller.angleDegrees();
    }

    @Override
    public void zeroYaw() {
        controller.zeroYaw();
    }

    @Override
    public void reset() {
        controller.reset();
    }

    @Override
    public void close() {
        controller.close();
    }

    private static AHRS createAhrs(ImuDevice device) {
        return new AHRS(comType(device), AHRS.NavXUpdateRate.k100Hz);
    }

    private static AHRS.NavXComType comType(ImuDevice device) {
        return switch (device.id()) {
            case 1 -> AHRS.NavXComType.kMXP_UART;
            case 2 -> AHRS.NavXComType.kUSB1;
            case 3 -> AHRS.NavXComType.kUSB2;
            case 4 -> AHRS.NavXComType.kI2C;
            default -> AHRS.NavXComType.kMXP_SPI;
        };
    }

    interface NavxController extends AutoCloseable {
        default void activate() {
            // default no-op
        }

        double yawDegrees();

        double angleDegrees();

        void zeroYaw();

        void reset();

        @Override
        void close();
    }

    private static final class AhrsNavxController implements NavxController {
        private final ImuDevice device;
        private AHRS ahrs;

        private AhrsNavxController(ImuDevice device) {
            this.device = Objects.requireNonNull(device, "device");
        }

        @Override
        public void activate() {
            ahrs();
        }

        @Override
        public double yawDegrees() {
            return ahrs().getYaw();
        }

        @Override
        public double angleDegrees() {
            return ahrs().getAngle();
        }

        @Override
        public void zeroYaw() {
            ahrs().zeroYaw();
        }

        @Override
        public void reset() {
            ahrs().reset();
        }

        @Override
        public void close() {
            if (ahrs != null) {
                ahrs.close();
            }
        }

        private AHRS ahrs() {
            if (ahrs == null) {
                ahrs = createAhrs(device);
            }
            return ahrs;
        }
    }
}
