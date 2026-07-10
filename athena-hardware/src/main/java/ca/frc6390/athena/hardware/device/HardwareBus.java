package ca.frc6390.athena.hardware.device;

import java.util.EnumSet;
import java.util.Objects;
import java.util.Set;

import ca.frc6390.athena.api.hardware.EncoderKind;
import ca.frc6390.athena.api.hardware.ImuKind;
import ca.frc6390.athena.api.hardware.MotorKind;

/**
 * Named hardware bus used to declare devices without repeating the bus on every ref.
 */
public record HardwareBus(String name, Set<HardwareInterface> interfaces) {
    public static HardwareBus rio() {
        return new HardwareBus("rio", EnumSet.allOf(HardwareInterface.class));
    }

    public static HardwareBus can(String name) {
        return new HardwareBus(name, EnumSet.of(HardwareInterface.CAN));
    }

    public HardwareBus {
        name = name == null || name.isBlank() ? "rio" : name;
        interfaces = interfaces == null || interfaces.isEmpty()
                ? Set.of(HardwareInterface.CAN)
                : Set.copyOf(interfaces);
    }

    public HardwareBus(String name) {
        this(name, Set.of(HardwareInterface.CAN));
    }

    public MotorDevice motor(MotorKind kind, int id) {
        return can(id).motor(kind);
    }

    public EncoderDevice encoder(EncoderKind kind, int id) {
        return can(id).encoder(kind);
    }

    public ImuDevice imu(ImuKind kind, int id) {
        return can(id).imu(kind);
    }

    public boolean supports(HardwareInterface hardwareInterface) {
        return interfaces.contains(Objects.requireNonNull(hardwareInterface, "hardwareInterface"));
    }

    public Connection can(int id) {
        return connect(new HardwareAddress.Can(id));
    }

    public Connection dio(int channel) {
        return connect(new HardwareAddress.Dio(channel));
    }

    public Connection quadrature(int channelA, int channelB) {
        return connect(new HardwareAddress.Quadrature(channelA, channelB, -1));
    }

    public Connection quadrature(int channelA, int channelB, int indexChannel) {
        return connect(new HardwareAddress.Quadrature(channelA, channelB, indexChannel));
    }

    public Connection analog(int channel) {
        return connect(new HardwareAddress.Analog(channel));
    }

    public Connection spi(SpiPort port) {
        return connect(new HardwareAddress.Spi(port));
    }

    public Connection i2c(I2cPort port) {
        return connect(new HardwareAddress.I2c(port, -1));
    }

    public Connection i2c(I2cPort port, int address) {
        return connect(new HardwareAddress.I2c(port, address));
    }

    public Connection serial(SerialPort port) {
        return connect(new HardwareAddress.Serial(port));
    }

    public Connection usb(int port) {
        return connect(new HardwareAddress.Usb(port));
    }

    private Connection connect(HardwareAddress address) {
        requireSupported(address);
        return new Connection(this, address);
    }

    private void requireSupported(HardwareAddress address) {
        Objects.requireNonNull(address, "address");
        if (!supports(address.hardwareInterface())) {
            throw new IllegalArgumentException(
                    "Hardware bus \"" + name + "\" does not provide " + address.hardwareInterface() + ".");
        }
    }

    /**
     * One physical connection bound to this hardware bus.
     */
    public static final class Connection {
        private final HardwareBus bus;
        private final HardwareAddress address;

        private Connection(HardwareBus bus, HardwareAddress address) {
            this.bus = Objects.requireNonNull(bus, "bus");
            this.address = Objects.requireNonNull(address, "address");
            bus.requireSupported(address);
        }

        public MotorDevice motor(MotorKind kind) {
            Objects.requireNonNull(kind, "kind");
            if (!(address instanceof HardwareAddress.Can can)) {
                throw new IllegalArgumentException("Motors require a CAN connection, not " + address.identity() + ".");
            }
            return MotorDevice.of(kind, can.id()).canbus(bus.name());
        }

        public EncoderDevice encoder(EncoderKind kind) {
            Objects.requireNonNull(kind, "kind");
            return EncoderDevice.connected(kind, bus.name(), address);
        }

        public ImuDevice imu(ImuKind kind) {
            Objects.requireNonNull(kind, "kind");
            return ImuDevice.connected(kind, bus.name(), address);
        }

        public DigitalInputDevice digitalInput() {
            return digitalInput(null);
        }

        public DigitalInputDevice digitalInput(String name) {
            if (!(address instanceof HardwareAddress.Dio dio)) {
                throw new IllegalArgumentException(
                        "Digital inputs require a DIO connection, not " + address.identity() + ".");
            }
            return new DigitalInputDevice(name, dio.channel(), false, null);
        }
    }
}
