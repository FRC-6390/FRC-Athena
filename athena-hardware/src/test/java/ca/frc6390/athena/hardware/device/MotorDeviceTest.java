package ca.frc6390.athena.hardware.device;

import edu.wpi.first.units.Units;
import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.api.hardware.MotorKinds;
import org.junit.jupiter.api.Test;

class MotorDeviceTest {
    @Test
    void newMotorDoesNotInventAControllerCurrentLimit() {
        MotorDevice motor = MotorDevice.of(MotorKinds.NEO, 1);

        assertEquals(0, motor.currentLimit().in(edu.wpi.first.units.Units.Amps));
        assertEquals(0, motor.supplyCurrentLimit().in(edu.wpi.first.units.Units.Amps));
        assertEquals(0, motor.statorCurrentLimit().in(edu.wpi.first.units.Units.Amps));
    }
}
