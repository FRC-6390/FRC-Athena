package ca.frc6390.athena.vendor.studica;

/**
 * Studica/NavX communication ports.
 */
public enum StudicaNavxPort {
    MXP_SPI(0),
    MXP_UART(1),
    USB1(2),
    USB2(3),
    I2C(4);

    private final int id;

    StudicaNavxPort(int id) {
        this.id = id;
    }

    /**
     * Returns the Athena declaration id for this NavX port.
     *
     * @return declaration id
     */
    public int id() {
        return id;
    }
}
