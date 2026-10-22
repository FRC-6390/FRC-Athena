package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.*;

import ca.frc6390.athena.hardware.backend.MotorHandle;
import ca.frc6390.athena.hardware.backend.MotorRuntimeConfig;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.api.hardware.MotorKinds;
import org.junit.jupiter.api.Test;

class RuntimeOverridesTest {
    @Test
    void repeatingAFailedPowerLimitRetriesHardwareAndSuccessfulRepeatsStayCached() {
        MotorDevice motor = MotorDevice.of(MotorKinds.KRAKEN_X60, 1);
        class FlakyMotor implements MotorHandle {
            int attempts;
            MotorRuntimeConfig applied;
            @Override public MotorDevice device() { return motor; }
            @Override public boolean supportsRuntimeConfiguration() { return true; }
            @Override public void applyRuntimeConfiguration(MotorRuntimeConfig config) {
                if (++attempts == 1) throw new IllegalStateException("CAN timeout");
                applied = config;
            }
        }
        FlakyMotor handle = new FlakyMotor();
        RuntimeOverrides overrides = new RuntimeOverrides();
        overrides.actionContext(new ActionContext() {
            @Override public MotorHandle motor(MotorDevice device) { return handle; }
        });

        overrides.supplyLimit(motor, 55);
        assertTrue(overrides.motorStatus(motor).startsWith("Error:"));
        assertNull(handle.applied);
        overrides.supplyLimit(motor, 55);
        assertEquals(2, handle.attempts);
        assertEquals(55, handle.applied.supplyCurrentLimitAmps());
        assertEquals("Applied", overrides.motorStatus(motor));
        overrides.supplyLimit(motor, 55);
        assertEquals(2, handle.attempts, "successful unchanged values must not reconfigure every write");
    }
}
