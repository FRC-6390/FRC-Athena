package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;

import java.util.Map;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class HookGroupTest {
    @Test
    void executesHookGroupsOwnedByNestedMechanisms() {
        boolean[] active = {false};
        AtomicInteger starts = new AtomicInteger();
        AtomicInteger activeCycles = new AtomicInteger();
        AtomicInteger ends = new AtomicInteger();
        AtomicInteger inactiveCycles = new AtomicInteger();
        Child child = new Child(active, starts, activeCycles, ends, inactiveCycles);
        MechanismScheduler scheduler = MechanismScheduler.create().register(new Parent(child));

        scheduler.robotPeriodic(0.00, 0.02);
        active[0] = true;
        scheduler.robotPeriodic(0.02, 0.02);
        scheduler.robotPeriodic(0.04, 0.02);
        active[0] = false;
        scheduler.robotPeriodic(0.06, 0.02);

        assertEquals(1, starts.get());
        assertEquals(2, activeCycles.get());
        assertEquals(1, ends.get());
        assertEquals(2, inactiveCycles.get());
    }

    private record Parent(Child child) implements Mechanism {
    }

    private static final class Child implements Mechanism {
        private final HookGroup controls;

        private Child(
                boolean[] active,
                AtomicInteger starts,
                AtomicInteger activeCycles,
                AtomicInteger ends,
                AtomicInteger inactiveCycles) {
            HookBinding binding = Events.when(() -> active[0]).active()
                    .onStart(starts::incrementAndGet)
                    .whileActive(activeCycles::incrementAndGet)
                    .onEnd(ends::incrementAndGet)
                    .whileInactive(inactiveCycles::incrementAndGet);
            controls = () -> Map.of("button", binding);
        }
    }
}
