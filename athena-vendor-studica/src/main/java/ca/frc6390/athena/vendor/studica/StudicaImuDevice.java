package ca.frc6390.athena.vendor.studica;

import ca.frc6390.athena.hardware.backend.ImuDevice;
import ca.frc6390.athena.hardware.imu.ImuSpec;
import com.studica.frc.AHRS;
import java.util.Objects;

/**
 * Studica/NavX IMU device backed by the Studica {@link AHRS} library.
 */
public final class StudicaImuDevice implements ImuDevice, AutoCloseable {
    private final ImuSpec spec;
    private final NavxController controller;

    /**
     * Creates a Studica/NavX IMU device using a real Studica AHRS.
     *
     * @param spec normalized IMU spec
     */
    public StudicaImuDevice(ImuSpec spec) {
        this(spec, new AhrsNavxController(createAhrs(spec)));
    }

    StudicaImuDevice(ImuSpec spec, NavxController controller) {
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
     * Returns accumulated angle in degrees.
     *
     * @return accumulated angle degrees
     */
    public double angleDegrees() {
        return controller.angleDegrees();
    }

    /**
     * Zeros the current yaw reading on the underlying NavX.
     */
    public void zeroYaw() {
        controller.zeroYaw();
    }

    /**
     * Resets accumulated NavX angle state.
     */
    public void reset() {
        controller.reset();
    }

    @Override
    public void close() {
        controller.close();
    }

    private static AHRS createAhrs(ImuSpec spec) {
        return new AHRS(comType(spec), AHRS.NavXUpdateRate.k100Hz);
    }

    private static AHRS.NavXComType comType(ImuSpec spec) {
        return switch (spec.id()) {
            case 1 -> AHRS.NavXComType.kMXP_UART;
            case 2 -> AHRS.NavXComType.kUSB1;
            case 3 -> AHRS.NavXComType.kUSB2;
            case 4 -> AHRS.NavXComType.kI2C;
            default -> AHRS.NavXComType.kMXP_SPI;
        };
    }

    interface NavxController extends AutoCloseable {
        double yawDegrees();

        double angleDegrees();

        void zeroYaw();

        void reset();

        @Override
        void close();
    }

    private static final class AhrsNavxController implements NavxController {
        private final AHRS ahrs;

        private AhrsNavxController(AHRS ahrs) {
            this.ahrs = Objects.requireNonNull(ahrs, "ahrs");
        }

        @Override
        public double yawDegrees() {
            return ahrs.getYaw();
        }

        @Override
        public double angleDegrees() {
            return ahrs.getAngle();
        }

        @Override
        public void zeroYaw() {
            ahrs.zeroYaw();
        }

        @Override
        public void reset() {
            ahrs.reset();
        }

        @Override
        public void close() {
            ahrs.close();
        }
    }
}
