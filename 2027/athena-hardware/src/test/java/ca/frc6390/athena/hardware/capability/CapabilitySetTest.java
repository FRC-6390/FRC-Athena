package ca.frc6390.athena.hardware.capability;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

class CapabilitySetTest {
    @Test
    void createsImmutableCapabilitySet() {
        CapabilitySet set = CapabilitySet.of(
                MotorCapability.PERCENT_OUTPUT,
                MotorCapability.VELOCITY_CLOSED_LOOP,
                null);

        assertTrue(set.contains(MotorCapability.PERCENT_OUTPUT));
        assertTrue(set.contains(MotorCapability.VELOCITY_CLOSED_LOOP));
        assertFalse(set.contains(MotorCapability.POSITION_CLOSED_LOOP));
        assertThrows(UnsupportedOperationException.class, () -> set.motorCapabilities().add(MotorCapability.CURRENT_LIMIT));
    }
}
