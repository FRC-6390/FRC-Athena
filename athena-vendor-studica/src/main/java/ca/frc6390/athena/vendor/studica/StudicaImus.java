package ca.frc6390.athena.vendor.studica;

import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.hardware.device.ImuDevice;

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
        return ImuDevice.of(ImuKinds.NAVX, safePort.id());
    }
}
