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
    private volatile double lastUpdateSeconds = Double.NaN;

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
        double[] acceleration = controller.linearAccelerationG();
        snapshot = new Snapshot(
                controller.yawDegrees(),
                controller.angleDegrees(),
                controller.pitchDegrees(),
                controller.rollDegrees(),
                controller.yawRateDegreesPerSecond(),
                controller.pitchRateDegreesPerSecond(),
                controller.rollRateDegreesPerSecond(),
                acceleration[0], acceleration[1], acceleration[2]);
        lastUpdateSeconds = System.nanoTime() * 1.0e-9;
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
    public double pitchRateDegreesPerSecond() {
        return snapshot.pitchRateDegreesPerSecond();
    }

    @Override
    public double rollRateDegreesPerSecond() {
        return snapshot.rollRateDegreesPerSecond();
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

    @Override public boolean isConnected() { return controller.isConnected(); }
    @Override public boolean isCalibrating() { return controller.isCalibrating(); }
    @Override public double lastUpdateSeconds() { return lastUpdateSeconds; }

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
            double pitchRateDegreesPerSecond,
            double rollRateDegreesPerSecond,
            double linearAccelerationXG,
            double linearAccelerationYG,
            double linearAccelerationZG) {
        private static Snapshot empty() {
            return new Snapshot(Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                    Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
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

        default double pitchRateDegreesPerSecond() { return 0.0; }

        default double rollRateDegreesPerSecond() { return 0.0; }

        double linearAccelerationXG();

        double linearAccelerationYG();

        double linearAccelerationZG();

        default double[] linearAccelerationG() {
            return new double[] {
                    linearAccelerationXG(),
                    linearAccelerationYG(),
                    linearAccelerationZG()
            };
        }

        default boolean isConnected() { return true; }

        default boolean isCalibrating() { return false; }

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
            return ahrs().getRawGyroZ();
        }

        @Override
        public double pitchRateDegreesPerSecond() {
            return ahrs().getRawGyroY();
        }

        @Override
        public double rollRateDegreesPerSecond() {
            return ahrs().getRawGyroX();
        }

        @Override
        public double linearAccelerationXG() {
            return sensorLinearAcceleration()[0];
        }

        @Override
        public double linearAccelerationYG() {
            return sensorLinearAcceleration()[1];
        }

        @Override
        public double linearAccelerationZG() {
            return sensorLinearAcceleration()[2];
        }

        @Override
        public double[] linearAccelerationG() {
            return sensorLinearAcceleration();
        }

        @Override public boolean isConnected() { return ahrs().isConnected(); }
        @Override public boolean isCalibrating() { return ahrs().isCalibrating(); }

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

        private double[] sensorLinearAcceleration() {
            AHRS sensor = ahrs();
            double x = sensor.getWorldLinearAccelX();
            double y = sensor.getWorldLinearAccelY();
            double z = sensor.getWorldLinearAccelZ();
            double roll = Math.toRadians(sensor.getRoll());
            double pitch = Math.toRadians(sensor.getPitch());
            double yaw = Math.toRadians(sensor.getYaw());
            double cr = Math.cos(roll), sr = Math.sin(roll);
            double cp = Math.cos(pitch), sp = Math.sin(pitch);
            double cy = Math.cos(yaw), sy = Math.sin(yaw);

            // NavX reports gravity-corrected acceleration in its world frame. Rotate it back
            // into sensor axes so ImuMount can consistently produce robot-frame acceleration.
            return new double[] {
                    cy * cp * x + sy * cp * y - sp * z,
                    (cy * sp * sr - sy * cr) * x
                            + (sy * sp * sr + cy * cr) * y + cp * sr * z,
                    (cy * sp * cr + sy * sr) * x
                            + (sy * sp * cr - cy * sr) * y + cp * cr * z
            };
        }
    }
}
