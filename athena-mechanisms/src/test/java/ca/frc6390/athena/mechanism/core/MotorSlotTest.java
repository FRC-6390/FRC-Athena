package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;

import ca.frc6390.athena.api.hardware.MotorKinds;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.MotorNeutralMode;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class MotorSlotTest {
    @Test
    void appliesTypedConfigurationWhenFilled() {
        Object owner = new Object();
        AtomicInteger configured = new AtomicInteger();
        MotorSlot<Object> slot = Slots.motor(owner, "drive", configured::incrementAndGet)
                .brake()
                .currentLimit(60)
                .inverted();

        Object returned = slot.fill(MotorDevice.of(MotorKinds.KRAKEN_X60, 1));

        assertSame(owner, returned);
        assertEquals(MotorNeutralMode.BRAKE, slot.get().neutralMode());
        assertEquals(60, slot.get().currentLimitAmps());
        assertEquals(true, slot.get().isInverted());
        assertEquals(1, configured.get());
    }

    @Test
    void laterSlotConfigurationUpdatesFilledDeviceAndOverridesEarlierValue() {
        AtomicInteger configured = new AtomicInteger();
        MotorSlot<Object> slot = Slots.motor(new Object(), "steer", configured::incrementAndGet)
                .brake();
        slot.fill(MotorDevice.of(MotorKinds.KRAKEN_X44, 2).coast());

        slot.coast().currentLimit(25);

        assertEquals(MotorNeutralMode.COAST, slot.get().neutralMode());
        assertEquals(25, slot.get().currentLimitAmps());
        assertEquals(3, configured.get());
    }
}
