package frc.robot.examples;

import ca.frc6390.athena.api.hardware.EncoderKinds;
import ca.frc6390.athena.api.hardware.ImuKinds;
import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.backend.FocPolicy;
import ca.frc6390.athena.hardware.device.HardwareBus;
import ca.frc6390.athena.hardware.device.I2cPort;
import ca.frc6390.athena.hardware.device.SerialPort;
import ca.frc6390.athena.hardware.device.SpiPort;
import ca.frc6390.athena.vendor.ctre.CtreMotorOptions;
import ca.frc6390.athena.vendor.rev.RevMotorOptions;
import ca.frc6390.athena.vendor.studica.StudicaImus;
import ca.frc6390.athena.vendor.studica.StudicaNavxPort;

/** Compile-checked declarations for every roboRIO connection family and vendor options. */
public final class HardwareConnections {
    public static final HardwareBus RIO = HardwareBus.rio();
    public static final HardwareBus CANIVORE = HardwareBus.can("canivore");

    public static final Object CAN_MOTOR = CANIVORE.motor(MotorKinds.KRAKEN_X60, 1)
            .vendor(CtreMotorOptions.class, options -> options
                    .supplyCurrentLimit(40)
                    .statorCurrentLimit(80)
                    .torqueCurrentLimit(100)
                    .foc(FocPolicy.ENABLE_IF_AVAILABLE));
    public static final Object REV_MOTOR = RIO.motor(MotorKinds.NEO, 2)
            .vendor(RevMotorOptions.class, options -> options
                    .smartCurrentLimit(35)
                    .voltageCompensation(12.0)
                    .openLoopRampSeconds(0.1)
                    .closedLoopRampSeconds(0.05)
                    .forwardSoftLimitRotations(100.0)
                    .reverseSoftLimitRotations(-2.0));
    public static final Object CAN_ENCODER = CANIVORE.encoder(EncoderKinds.CANCODER, 3);
    public static final Object PWM_ENCODER = RIO.dio(0).encoder(EncoderKinds.REV_THROUGH_BORE_V2);
    public static final Object QUADRATURE_ENCODER = RIO.quadrature(1, 2, 3)
            .encoder(EncoderKinds.REV_THROUGH_BORE_QUADRATURE);
    public static final Object PIGEON = CANIVORE.imu(ImuKinds.PIGEON_2, 4);
    public static final Object NAVX_SPI = StudicaImus.navx(StudicaNavxPort.MXP_SPI);
    public static final Object DIGITAL_INPUT = RIO.dio(4).digitalInput("piece sensor");

    // Connections without a built-in Athena sensor kind can still be passed to custom backends.
    public static final HardwareBus.Connection ANALOG = RIO.analog(0);
    public static final HardwareBus.Connection SPI = RIO.spi(SpiPort.ONBOARD_CS0);
    public static final HardwareBus.Connection I2C = RIO.i2c(I2cPort.MXP, 0x42);
    public static final HardwareBus.Connection SERIAL = RIO.serial(SerialPort.MXP);
    public static final HardwareBus.Connection USB = RIO.usb(1);

    private HardwareConnections() {
    }
}
