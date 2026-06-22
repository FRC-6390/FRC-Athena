package ca.frc6390.athena.hardware.spec;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaMotor;

class MotorSpecTest {
    @Test
    void normalizesDefaultsAndPath() {
        MotorSpec spec = new MotorSpec(
                "",
                "",
                AthenaMotor.SIM,
                1,
                "",
                null,
                40,
                false);

        assertEquals("robot", spec.ownerPath());
        assertEquals("motor", spec.name());
        assertEquals("rio", spec.canbus());
        assertEquals(NeutralMode.COAST, spec.neutralMode());
        assertEquals("robot.motor", spec.path());
    }
}
