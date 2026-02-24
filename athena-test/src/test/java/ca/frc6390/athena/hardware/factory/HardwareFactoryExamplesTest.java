package ca.frc6390.athena.hardware.factory;

import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.UUID;
import java.util.NoSuchElementException;

import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.examples.HardwareFactoryExamples;
import ca.frc6390.athena.hardware.imu.ImuConfig;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import org.junit.jupiter.api.Test;

final class HardwareFactoryExamplesTest {

    @Test
    void registriesExposeClearMissingProviderErrors() {
        String key = "missing.vendor." + UUID.randomUUID();

        NoSuchElementException motorEx =
                assertThrows(NoSuchElementException.class, () -> HardwareFactoryExamples.motorConfig(key, 1));
        assertTrue(motorEx.getMessage().contains("Missing provider for motor '" + key + "'"));

        NoSuchElementException encoderEx =
                assertThrows(NoSuchElementException.class, () -> HardwareFactoryExamples.encoderConfig(key, 2));
        assertTrue(encoderEx.getMessage().contains("Missing provider for encoder '" + key + "'"));

        NoSuchElementException imuEx =
                assertThrows(NoSuchElementException.class, () -> HardwareFactoryExamples.imuConfig(key, 3));
        assertTrue(imuEx.getMessage().contains("Missing provider for IMU '" + key + "'"));
    }

    @Test
    void factoryHelpersRejectUnknownTypeKeys() {
        MotorControllerConfig motorConfig = MotorControllerConfig.create(() -> "test.missing.motor", 1);
        EncoderConfig encoderConfig = EncoderConfig.create(() -> "test.missing.encoder", 2);
        ImuConfig imuConfig = ImuConfig.create(() -> "test.missing.imu", 3);

        IllegalArgumentException motorEx =
                assertThrows(IllegalArgumentException.class, () -> HardwareFactoryExamples.createMotor(motorConfig));
        assertTrue(motorEx.getMessage().contains("No motor factory for type: test.missing.motor"));

        IllegalArgumentException encoderEx =
                assertThrows(IllegalArgumentException.class, () -> HardwareFactoryExamples.createEncoder(encoderConfig));
        assertTrue(encoderEx.getMessage().contains("No encoder factory for type: test.missing.encoder"));

        IllegalArgumentException imuEx =
                assertThrows(IllegalArgumentException.class, () -> HardwareFactoryExamples.createImu(imuConfig));
        assertTrue(imuEx.getMessage().contains("No IMU factory for type: test.missing.imu"));
    }
}
