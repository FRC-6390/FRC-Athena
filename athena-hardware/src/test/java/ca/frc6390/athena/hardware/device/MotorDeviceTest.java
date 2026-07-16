package ca.frc6390.athena.hardware.device;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.MotorKinds;
import org.junit.jupiter.api.Test;

class MotorDeviceTest {
    @Test
    void newMotorDoesNotInventAControllerCurrentLimit() {
        MotorDevice motor = MotorDevice.of(MotorKinds.NEO, 1);

        assertEquals(0, motor.currentLimitAmps());
        assertEquals(0, motor.supplyCurrentLimitAmps());
        assertEquals(0, motor.statorCurrentLimitAmps());
    }
}
