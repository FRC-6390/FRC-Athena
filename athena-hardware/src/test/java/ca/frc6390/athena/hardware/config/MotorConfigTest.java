package ca.frc6390.athena.hardware.config;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.api.hardware.AthenaMotor;
import ca.frc6390.athena.api.hardware.MotorId;
import ca.frc6390.athena.hardware.spec.NeutralMode;

class MotorConfigTest {
    @Test
    void lowersExplicitHardwareToSpec() {
        var spec = MotorConfig.create()
                .hardware(AthenaMotor.SIM, 4)
                .canbus("canivore")
                .brake()
                .currentLimit(35)
                .integratedEncoder()
                .toSpec("intake", "roller");

        assertEquals("intake.roller", spec.path());
        assertEquals(AthenaMotor.SIM, spec.kind());
        assertEquals(4, spec.id());
        assertEquals("canivore", spec.canbus());
        assertEquals(NeutralMode.BRAKE, spec.neutralMode());
        assertEquals(35, spec.currentLimitAmps());
        assertTrue(spec.integratedEncoder());
    }

    @Test
    void lowersHardwareAliasToSpec() {
        MotorId id = MotorId.of(AthenaMotor.TALON_FX, 12).canbus("canivore");

        var spec = MotorConfig.create()
                .hardware(id)
                .toSpec("shooter", "leader");

        assertEquals(AthenaMotor.TALON_FX, spec.kind());
        assertEquals(12, spec.id());
        assertEquals("canivore", spec.canbus());
    }

    @Test
    void requiresHardwareKindBeforeLowering() {
        assertThrows(IllegalStateException.class, () -> MotorConfig.create().toSpec("bad", "motor"));
    }

    @Test
    void storesTypedVendorOptionsWithoutVendorImportsInSpec() {
        var spec = MotorConfig.create()
                .hardware(AthenaMotor.TALON_FX, 1)
                .vendor(TestMotorOptions.class, options -> options.limit = 80)
                .toSpec("shooter", "leader");

        assertEquals(80, spec.vendorOptions().find(TestMotorOptions.class).orElseThrow().limit);
    }

    public static final class TestMotorOptions {
        public int limit;
    }
}
