package ca.frc6390.athena.vendor.studica;

import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.HardwareAddress;
import com.studica.frc.AHRS;
import java.util.Objects;

/**
 * Studica/NavX IMU handle backed by the Studica {@link AHRS} library.
 */
public final class StudicaImuHandle implements ImuHandle, AutoCloseable {
    private final ImuDevice device;
    private final NavxController controller;
    private volatile Snapshot snapshot = Snapshot.empty();

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
        return snapshot.yawDegrees();
    }

    @Override
    public void activate() {
        controller.activate();
    }

    @Override
    public void refreshInputs() {
        snapshot = new Snapshot(
                controller.yawDegrees(),
                controller.angleDegrees(),
                controller.pitchDegrees(),
                controller.rollDegrees(),
                controller.yawRateDegreesPerSecond(),
                controller.linearAccelerationXG(),
                controller.linearAccelerationYG(),
                controller.linearAccelerationZG());
    }

    @Override
    public double angleDegrees() {
        return snapshot.angleDegrees();
    }

    @Override
    public double pitchDegrees() {
        return snapshot.pitchDegrees();
    }

    @Override
    public double rollDegrees() {
        return snapshot.rollDegrees();
    }

    @Override
    public double yawRateDegreesPerSecond() {
        return snapshot.yawRateDegreesPerSecond();
    }

    @Override
    public double linearAccelerationXG() {
        return snapshot.linearAccelerationXG();
    }

    @Override
    public double linearAccelerationYG() {
        return snapshot.linearAccelerationYG();
    }

    @Override
    public double linearAccelerationZG() {
        return snapshot.linearAccelerationZG();
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

    private record Snapshot(
            double yawDegrees,
            double angleDegrees,
            double pitchDegrees,
            double rollDegrees,
            double yawRateDegreesPerSecond,
            double linearAccelerationXG,
            double linearAccelerationYG,
            double linearAccelerationZG) {
        private static Snapshot empty() {
            return new Snapshot(Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                    Double.NaN, Double.NaN, Double.NaN, Double.NaN);
        }
    }

    private static AHRS.NavXComType comType(ImuDevice device) {
        HardwareAddress address = device.connection();
        if (address instanceof HardwareAddress.Spi) {
            return AHRS.NavXComType.kMXP_SPI;
        }
        if (address instanceof HardwareAddress.Serial) {
            return AHRS.NavXComType.kMXP_UART;
        }
        if (address instanceof HardwareAddress.I2c) {
            return AHRS.NavXComType.kI2C;
        }
        if (address instanceof HardwareAddress.Usb usb && usb.port() == 1) {
            return AHRS.NavXComType.kUSB1;
        }
        if (address instanceof HardwareAddress.Usb usb && usb.port() == 2) {
            return AHRS.NavXComType.kUSB2;
        }
        throw new IllegalArgumentException("NavX does not support " + address.identity() + ".");
    }

    interface NavxController extends AutoCloseable {
        default void activate() {
            // default no-op
        }

        double yawDegrees();

        double angleDegrees();

        double pitchDegrees();

        double rollDegrees();

        double yawRateDegreesPerSecond();

        double linearAccelerationXG();

        double linearAccelerationYG();

        double linearAccelerationZG();

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
        public double pitchDegrees() {
            return ahrs().getPitch();
        }

        @Override
        public double rollDegrees() {
            return ahrs().getRoll();
        }

        @Override
        public double yawRateDegreesPerSecond() {
            return ahrs().getRate();
        }

        @Override
        public double linearAccelerationXG() {
            return ahrs().getWorldLinearAccelX();
        }

        @Override
        public double linearAccelerationYG() {
            return ahrs().getWorldLinearAccelY();
        }

        @Override
        public double linearAccelerationZG() {
            return ahrs().getWorldLinearAccelZ();
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
