package ca.frc6390.athena.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotSame;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.PathAction;
import ca.frc6390.athena.mechanism.core.Paths;
import java.util.List;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class AutoRuntimeTest {
    @Test
    void indexesNamesAndRejectsDuplicates() {
        Action action = Actions.waitSeconds(1.0);
        AutoRuntime runtime = Autos.runtime(Autos.routine(" score ", action), Autos.routine("leave", action));
        assertEquals(List.of("score", "leave"), List.copyOf(runtime.routineNames()));
        assertEquals("score", runtime.selectedName());
        assertTrue(runtime.find(" score ").isPresent());
        assertThrows(IllegalArgumentException.class,
                () -> Autos.runtime(Autos.routine("score", action), Autos.routine(" score ", action)));
    }

    @Test
    void selectedActionIsCreatedOncePerRunAndOwnedByMechanismScheduler() {
        AtomicInteger creations = new AtomicInteger();
        AutoRuntime runtime = Autos.runtime(Autos.routine("auto", () -> {
            creations.incrementAndGet();
            return Actions.waitSeconds(1.0);
        }));
        Action first = runtime.initialize();
        assertSame(first, runtime.initialize());
        assertSame(first, runtime.end().orElseThrow());
        Action second = runtime.initialize();
        assertNotSame(first, second);
        assertEquals(2, creations.get());
    }

    @Test
    void pathActionsCarryNativeSplitsMarkersAndOdometryReset() {
        Action shoot = Actions.waitSeconds(0.5);
        PathAction path = Paths.choreo("Demo")
                .split(2)
                .resetOdometry()
                .marker(" Shoot ", shoot);
        assertEquals("choreo:Demo:split:2", path.key());
        assertEquals(2, path.splitIndex());
        assertTrue(path.resetsOdometry());
        assertSame(shoot, path.markers().get("Shoot"));
        assertThrows(IllegalArgumentException.class, () -> path.marker("Shoot", shoot));
    }
}
