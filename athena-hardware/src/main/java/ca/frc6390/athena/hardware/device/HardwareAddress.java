package ca.frc6390.athena.hardware.device;

import java.util.Objects;

/**
 * Typed physical connection on a {@link HardwareBus}.
 */
public sealed interface HardwareAddress permits HardwareAddress.Can, HardwareAddress.Dio,
        HardwareAddress.Quadrature, HardwareAddress.Analog, HardwareAddress.Spi,
        HardwareAddress.I2c, HardwareAddress.Serial, HardwareAddress.Usb {
    /** Returns the physical interface required by this address. */
    HardwareInterface hardwareInterface();

    /** Returns the primary numeric address used in hardware identity keys. */
    int primaryAddress();

    /** Returns the stable connection detail used in diagnostics and identity keys. */
    String identity();

    record Can(int id) implements HardwareAddress {
        public Can {
            requireNonNegative(id, "CAN device ID");
        }

        @Override
        public HardwareInterface hardwareInterface() {
            return HardwareInterface.CAN;
        }

        @Override
        public int primaryAddress() {
            return id;
        }

        @Override
        public String identity() {
            return "can";
        }
    }

    record Dio(int channel) implements HardwareAddress {
        public Dio {
            requireNonNegative(channel, "DIO channel");
        }

        @Override
        public HardwareInterface hardwareInterface() {
            return HardwareInterface.DIO;
        }

        @Override
        public int primaryAddress() {
            return channel;
        }

        @Override
        public String identity() {
            return "dio";
        }
    }

    record Quadrature(int channelA, int channelB, int indexChannel) implements HardwareAddress {
        public Quadrature {
            requireNonNegative(channelA, "Quadrature channel A");
            requireNonNegative(channelB, "Quadrature channel B");
            if (indexChannel < -1) {
                throw new IllegalArgumentException("Quadrature index channel must be non-negative when provided.");
            }
        }

        @Override
        public HardwareInterface hardwareInterface() {
            return HardwareInterface.DIO;
        }

        @Override
        public int primaryAddress() {
            return channelA;
        }

        @Override
        public String identity() {
            String index = indexChannel < 0 ? "" : ":" + indexChannel;
            return "quadrature:" + channelA + ":" + channelB + index;
        }
    }

    record Analog(int channel) implements HardwareAddress {
        public Analog {
            requireNonNegative(channel, "Analog channel");
        }

        @Override
        public HardwareInterface hardwareInterface() {
            return HardwareInterface.ANALOG;
        }

        @Override
        public int primaryAddress() {
            return channel;
        }

        @Override
        public String identity() {
            return "analog";
        }
    }

    record Spi(SpiPort port) implements HardwareAddress {
        public Spi {
            Objects.requireNonNull(port, "port");
        }

        @Override
        public HardwareInterface hardwareInterface() {
            return HardwareInterface.SPI;
        }

        @Override
        public int primaryAddress() {
            return port.ordinal();
        }

        @Override
        public String identity() {
            return "spi:" + port.name().toLowerCase(java.util.Locale.ROOT);
        }
    }

    record I2c(I2cPort port, int address) implements HardwareAddress {
        public I2c {
            Objects.requireNonNull(port, "port");
            if (address < -1) {
                throw new IllegalArgumentException("I2C address must be non-negative when provided.");
            }
        }

        @Override
        public HardwareInterface hardwareInterface() {
            return HardwareInterface.I2C;
        }

        @Override
        public int primaryAddress() {
            return address < 0 ? port.ordinal() : address;
        }

        @Override
        public String identity() {
            String deviceAddress = address < 0 ? "" : ":" + address;
            return "i2c:" + port.name().toLowerCase(java.util.Locale.ROOT) + deviceAddress;
        }
    }

    record Serial(SerialPort port) implements HardwareAddress {
        public Serial {
            Objects.requireNonNull(port, "port");
        }

        @Override
        public HardwareInterface hardwareInterface() {
            return HardwareInterface.SERIAL;
        }

        @Override
        public int primaryAddress() {
            return port.ordinal();
        }

        @Override
        public String identity() {
            return "serial:" + port.name().toLowerCase(java.util.Locale.ROOT);
        }
    }

    record Usb(int port) implements HardwareAddress {
        public Usb {
            requireNonNegative(port, "USB port");
        }

        @Override
        public HardwareInterface hardwareInterface() {
            return HardwareInterface.USB;
        }

        @Override
        public int primaryAddress() {
            return port;
        }

        @Override
        public String identity() {
            return "usb";
        }
    }

    private static void requireNonNegative(int value, String name) {
        if (value < 0) {
            throw new IllegalArgumentException(name + " must be non-negative.");
        }
    }
}
