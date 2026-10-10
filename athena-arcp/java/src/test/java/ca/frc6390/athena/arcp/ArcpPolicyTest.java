package ca.frc6390.athena.arcp;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.nio.file.Path;
import java.time.Duration;
import org.junit.jupiter.api.Test;

class ArcpPolicyTest {
    @Test
    void defaultsEnableArcpOnFieldLegalPortsWithNoBudget() {
        ArcpPolicy policy = ArcpPolicy.of();
        assertTrue(policy.enabled());
        assertEquals(5808, ArcpPolicy.DEFAULT_CONTROL_PORT);
        assertEquals(5809, ArcpPolicy.DEFAULT_REALTIME_PORT);
        assertEquals(ArcpPolicy.DEFAULT_CONTROL_PORT, policy.controlPort());
        assertEquals(ArcpPolicy.DEFAULT_REALTIME_PORT, policy.realtimePort());
        assertEquals(0L, policy.bandwidthBudgetBytesPerSecond());
        assertEquals(ArcpPolicy.DEFAULT_MTU, policy.mtu());
        assertEquals(Duration.ofSeconds(1), policy.keyframeInterval());
        assertFalse(policy.storeDirectory().isPresent());
    }

    @Test
    void settersReturnNewImmutablePoliciesWithValidation() {
        ArcpPolicy base = ArcpPolicy.of();
        ArcpPolicy tuned = base.controlPort(5805).realtimePort(5806).bandwidthBudgetBytesPerSecond(400_000).mtu(1400)
                .storeDirectory(Path.of("/tmp/store")).modelDirectory(Path.of("/tmp/models"));
        assertEquals(ArcpPolicy.DEFAULT_CONTROL_PORT, base.controlPort());
        assertEquals(5805, tuned.controlPort());
        assertEquals(5806, tuned.realtimePort());
        assertEquals(ArcpPolicy.DEFAULT_REALTIME_PORT, base.realtimePort());
        assertEquals(400_000L, tuned.bandwidthBudgetBytesPerSecond());
        assertEquals(1400, tuned.mtu());
        assertEquals(Path.of("/tmp/store"), tuned.storeDirectory().orElseThrow());
        assertEquals(Path.of("/tmp/models"), tuned.modelDirectory().orElseThrow());

        assertThrows(IllegalArgumentException.class, () -> base.controlPort(70_000));
        assertThrows(IllegalArgumentException.class, () -> base.realtimePort(-1));
        assertThrows(IllegalArgumentException.class, () -> base.mtu(64));
        assertThrows(IllegalArgumentException.class, () -> base.bandwidthBudgetBytesPerSecond(-1));
        assertThrows(IllegalArgumentException.class, () -> base.keyframeInterval(Duration.ZERO));
    }

    @Test
    void disabledPolicyIsDisabled() {
        assertFalse(ArcpPolicy.disabled().enabled());
    }

    @Test
    void cycleStoresPrimitivesWithoutEntryObjectsAndClearsForReuse() {
        ArcpCycle cycle = new ArcpCycle();
        for (int round = 0; round < 3; round++) {
            cycle.clear();
            for (int i = 0; i < 100; i++) {
                cycle.setNumber(i + 1, i * 0.5);
            }
            cycle.setBoolean(200, true).setLong(201, 7L).set(202, "text");
            assertEquals(103, cycle.size());
            assertEquals(1L, cycle.signalId(0));
            assertEquals(0.5, (Double) cycle.value(1), 1e-12);
            assertEquals(Boolean.TRUE, cycle.value(100));
            assertEquals(7L, cycle.value(101));
            assertEquals("text", cycle.value(102));
        }
        cycle.clear();
        assertEquals(0, cycle.size());
    }

    @Test
    void cycleRejectsNullValues() {
        ArcpCycle cycle = new ArcpCycle();
        assertThrows(NullPointerException.class, () -> cycle.set(1L, null));
        assertThrows(IllegalArgumentException.class, () -> cycle.set(1L, new Object()));
    }
}
