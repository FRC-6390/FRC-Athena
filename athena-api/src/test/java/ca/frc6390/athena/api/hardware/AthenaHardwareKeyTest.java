package ca.frc6390.athena.api.hardware;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertInstanceOf;

import org.junit.jupiter.api.Test;

class AthenaHardwareKeyTest {
    @Test
    void motorKeysAreStableAndNamespaced() {
        assertEquals("ctre:talon-fx", AthenaMotor.TALON_FX.key());
        assertEquals("ctre:kraken-x60", AthenaMotor.KRAKEN_X60.key());
        assertEquals("ctre:kraken-x44", AthenaMotor.KRAKEN_X44.key());
        assertEquals("rev:spark-max-brushless", AthenaMotor.SPARK_MAX_BRUSHLESS.key());
        assertEquals("sim:motor", AthenaMotor.SIM.key());
        assertInstanceOf(MotorKind.class, AthenaMotor.TALON_FX);
        assertInstanceOf(HardwareKind.class, AthenaMotor.TALON_FX);
    }

    @Test
    void encoderKeysAreStableAndNamespaced() {
        assertEquals("ctre:cancoder", AthenaEncoder.CANCODER.key());
        assertEquals("athena:integrated-motor", AthenaEncoder.INTEGRATED_MOTOR.key());
        assertEquals("sim:encoder", AthenaEncoder.SIM.key());
        assertInstanceOf(EncoderKind.class, AthenaEncoder.CANCODER);
        assertInstanceOf(HardwareKind.class, AthenaEncoder.CANCODER);
    }

    @Test
    void imuKeysAreStableAndNamespaced() {
        assertEquals("ctre:pigeon-2", AthenaImu.PIGEON_2.key());
        assertEquals("studica:navx", AthenaImu.NAVX.key());
        assertEquals("sim:imu", AthenaImu.SIM.key());
        assertInstanceOf(ImuKind.class, AthenaImu.PIGEON_2);
        assertInstanceOf(HardwareKind.class, AthenaImu.PIGEON_2);
    }

    @Test
    void cameraKeysAreStableAndNamespaced() {
        assertEquals("photonvision:camera", AthenaCamera.PHOTONVISION.key());
        assertEquals("limelight:camera", AthenaCamera.LIMELIGHT.key());
        assertEquals("helios:camera", AthenaCamera.HELIOS.key());
        assertEquals("sim:camera", AthenaCamera.SIM.key());
        assertInstanceOf(CameraKind.class, AthenaCamera.PHOTONVISION);
        assertInstanceOf(HardwareKind.class, AthenaCamera.PHOTONVISION);
    }
}
