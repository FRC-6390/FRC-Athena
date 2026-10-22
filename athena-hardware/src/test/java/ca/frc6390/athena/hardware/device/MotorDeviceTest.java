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

    @Test
    void revPortableLimitRemainsPhaseWhileExplicitSupplyKeepsItsOwnDomain() {
        MotorDevice motor = MotorDevice.of(MotorKinds.NEO, 2).currentLimit(Units.Amps.of(60))
                .supplyCurrentLimit(Units.Amps.of(10)).statorCurrentLimit(Units.Amps.of(50));
        assertEquals(10.0, motor.currentLimits().supply().in(Units.Amps));
        assertEquals(50.0, motor.currentLimits().stator().in(Units.Amps));
        MotorDevice onlySupply = MotorDevice.of(MotorKinds.NEO, 3).supplyCurrentLimit(Units.Amps.of(10));
        assertEquals(10.0, onlySupply.currentLimits().supply().in(Units.Amps));
        assertEquals(0.0, onlySupply.currentLimits().stator().in(Units.Amps));
        MotorDevice portable = MotorDevice.of(MotorKinds.NEO, 4).currentLimit(Units.Amps.of(35));
        assertEquals(0.0, portable.currentLimits().supply().in(Units.Amps));
        assertEquals(35.0, portable.currentLimits().stator().in(Units.Amps));
    }
}
