package ca.frc6390.athena.hardware.device;

import edu.wpi.first.units.Units;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import org.junit.jupiter.api.Test;

class MotorConstantsTest {
    @Test
    void stallAndFreePointsDeriveResistanceTorqueConstantAndSpeedConstant() {
        MotorConstants motor = MotorConstants.of(Units.NewtonMeters.of(2.0), Units.Amps.of(100.0), Units.Amps.of(2.0), Units.RPM.of(6000.0));

        assertEquals(0.12, motor.resistance().in(Units.Ohms), 1.0e-12);
        assertEquals(0.02, motor.torqueConstantNmPerAmp(), 1.0e-12);
        double freeSpeed = 6000.0 * 2.0 * Math.PI / 60.0;
        assertEquals(freeSpeed / (12.0 - 0.12 * 2.0), motor.speedConstantRadPerSecPerVolt(), 1.0e-9);
        assertEquals(freeSpeed, motor.freeSpeed().in(Units.RadiansPerSecond), 1.0e-9);
    }

    @Test
    void expectedUnloadedSpeedAtFreeCurrentMatchesFreeSpeed() {
        MotorConstants motor = MotorConstants.of(Units.NewtonMeters.of(2.0), Units.Amps.of(100.0), Units.Amps.of(2.0), Units.RPM.of(6000.0));

        assertEquals(motor.freeSpeed().in(Units.RadiansPerSecond), motor.expectedUnloadedSpeedRadPerSec(12.0), 1.0e-9);
        assertEquals(0.0, motor.expectedUnloadedSpeedRadPerSec(0.24), 1.0e-9);
    }

    @Test
    void builtInConstantsExistForKnownMotorsOnly() {
        assertTrue(MotorConstants.forKind(MotorKinds.KRAKEN_X60).isPresent());
        assertTrue(MotorConstants.forKind(MotorKinds.NEO).isPresent());
        assertTrue(MotorConstants.forKind(MotorKinds.NEO.controlledBy(
                ca.frc6390.athena.api.hardware.MotorControllerKinds.SPARK_MAX)).isPresent());
        assertFalse(MotorConstants.forKind(MotorKinds.MINION).isPresent());
        assertFalse(MotorConstants.forKind(null).isPresent());
        assertFalse(MotorConstants.forKind(() -> "custom:motor").isPresent());
    }

    @Test
    void rejectsNonPhysicalValues() {
        assertThrows(IllegalArgumentException.class, () -> MotorConstants.of(Units.NewtonMeters.of(0.0), Units.Amps.of(100.0), Units.Amps.of(2.0), Units.RPM.of(6000.0)));
        assertThrows(IllegalArgumentException.class, () -> MotorConstants.of(Units.NewtonMeters.of(2.0), Units.Amps.of(100.0), Units.Amps.of(120.0), Units.RPM.of(6000.0)));
    }
}
