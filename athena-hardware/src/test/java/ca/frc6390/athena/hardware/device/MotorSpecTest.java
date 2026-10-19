package ca.frc6390.athena.hardware.device;

import edu.wpi.first.units.Units;
import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.power.MotorThermalSpec;
import org.junit.jupiter.api.Test;

/** One motor table feeds the electrical and the thermal model, so their shared values cannot drift apart. */
class MotorSpecTest {
    @Test
    void thermalResistanceAndSpeedConstantComeFromTheElectricalConstants() {
        for (MotorKinds kind : MotorKinds.values()) {
            MotorSpec spec = MotorSpec.forKind(kind).orElse(null);
            if (spec == null) {
                continue;
            }
            MotorConstants constants = MotorConstants.forKind(kind).orElseThrow();
            MotorThermalSpec thermal = MotorThermalSpec.forKind(kind);
            assertEquals(constants.resistance().in(Units.Ohms), thermal.resistanceOhms(), 1.0e-12, kind.name());
            assertEquals(constants.speedConstantRadPerSecPerVolt(), thermal.kvRadPerSecondPerVolt(), 1.0e-12, kind.name());
            assertEquals(spec.maxWindingCelsius(), thermal.maxWindingCelsius(), 0.0, kind.name());
        }
    }

    @Test
    void krakenX60UsesTheSharedRowAndKvFromFreeSpeed() {
        MotorConstants constants = MotorConstants.forKind(MotorKinds.KRAKEN_X60).orElseThrow();
        // Free speed 5800 rpm (607 rad/s) over the 11.93 V back-EMF left after the free-run winding drop: ~50.9 rad/s per volt.
        assertEquals(50.89, constants.speedConstantRadPerSecPerVolt(), 0.05);
        assertEquals(12.0 / 366.0, constants.resistance().in(Units.Ohms), 1.0e-9);
    }

    @Test
    void kindsWithoutARowUseTheGenericThermalSpec() {
        assertTrue(MotorSpec.forKind(MotorKinds.MINION).isEmpty());
        assertSame(MotorThermalSpec.GENERIC, MotorThermalSpec.forKind(MotorKinds.MINION));
        assertTrue(MotorConstants.forKind(MotorKinds.MINION).isEmpty());
    }
}
