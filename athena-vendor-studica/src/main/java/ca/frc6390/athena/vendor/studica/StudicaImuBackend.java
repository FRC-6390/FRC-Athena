package ca.frc6390.athena.vendor.studica;

import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.hardware.backend.ImuBackend;
import ca.frc6390.athena.hardware.backend.ImuHandle;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.HardwarePort;
import ca.frc6390.athena.hardware.device.I2cPort;
import ca.frc6390.athena.hardware.device.SerialPort;
import ca.frc6390.athena.hardware.device.SpiPort;
import java.util.Objects;
import java.util.function.Function;

/**
 * Studica/NavX IMU backend backed by the Studica AHRS library.
 */
public final class StudicaImuBackend implements ImuBackend {
    private final Function<ImuDevice, ImuHandle> handleFactory;

    /**
     * Creates a backend that constructs real Studica/NavX devices.
     */
    public StudicaImuBackend() {
        this(StudicaImuHandle::new);
    }

    StudicaImuBackend(Function<ImuDevice, ImuHandle> handleFactory) {
        this.handleFactory = Objects.requireNonNull(handleFactory, "handleFactory");
    }

    @Override
    public boolean supports(ImuKind kind) {
        return kind == ImuKinds.NAVX || kind.key().equals("studica:navx");
    }

    @Override
    public boolean supports(ImuDevice device) {
        if (!supports(device.kind())) {
            return false;
        }
        HardwarePort port = device.port();
        if (port instanceof HardwarePort.Spi spi) {
            return spi.port() == SpiPort.MXP;
        }
        if (port instanceof HardwarePort.Serial serial) {
            return serial.port() == SerialPort.MXP;
        }
        if (port instanceof HardwarePort.I2c i2c) {
            return i2c.port() == I2cPort.MXP;
        }
        return port instanceof HardwarePort.Usb usb && (usb.port() == 1 || usb.port() == 2);
    }

    @Override
    public ImuHandle create(ImuDevice device) {
        return handleFactory.apply(device);
    }
}
