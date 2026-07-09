package ca.frc6390.athena.vendor.studica;

import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.HardwareBus;
import ca.frc6390.athena.hardware.device.HardwarePort;
import ca.frc6390.athena.hardware.device.I2cPort;
import ca.frc6390.athena.hardware.device.SerialPort;
import ca.frc6390.athena.hardware.device.SpiPort;

/**
 * Studica IMU declaration helpers.
 */
public final class StudicaImus {
    private StudicaImus() {
    }

    /**
     * Creates a NavX declaration for a named port.
     *
     * @param port NavX communication port
     * @return IMU declaration
     */
    public static ImuDevice navx(StudicaNavxPort port) {
        StudicaNavxPort safePort = port == null ? StudicaNavxPort.MXP_SPI : port;
        HardwarePort connection = switch (safePort) {
            case MXP_SPI -> HardwarePort.spi(SpiPort.MXP);
            case MXP_UART -> HardwarePort.serial(SerialPort.MXP);
            case USB1 -> HardwarePort.usb(1);
            case USB2 -> HardwarePort.usb(2);
            case I2C -> HardwarePort.i2c(I2cPort.MXP);
        };
        return HardwareBus.rio().imu(ImuKinds.NAVX, connection);
    }
}
