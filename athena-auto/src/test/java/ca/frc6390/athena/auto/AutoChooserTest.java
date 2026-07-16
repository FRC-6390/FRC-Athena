package ca.frc6390.athena.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertSame;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import java.util.concurrent.atomic.AtomicInteger;
import org.junit.jupiter.api.Test;

class AutoChooserTest {
    @Test
    void selectionIsInertAndDoesNotReplaceTheRunningAuto() {
        AtomicInteger executions = new AtomicInteger();
        Action first = Actions.doOnce(executions::incrementAndGet);
        Action second = Actions.doOnce(() -> executions.addAndGet(10));
        AutoChooser chooser = Autos.chooser("Autos")
                .defaultAuto("First", first)
                .auto("Second", second);

        chooser.select("Second");
        assertEquals(0, executions.get());
        assertTrue(chooser.runningAction().isEmpty());

        assertSame(second, chooser.start());
        chooser.select("First");

        assertSame(second, chooser.runningAction().orElseThrow());
        assertEquals("First", chooser.selectedName());
        assertEquals("Second", chooser.runningName().orElseThrow());
    }

    @Test
    void validatesDuplicateAndUnknownNames() {
        Action action = Actions.waitSeconds(1.0);
        AutoChooser chooser = Autos.chooser("Autos").defaultAuto("Score", action);

        assertThrows(IllegalArgumentException.class, () -> chooser.auto(" Score ", action));
        assertThrows(IllegalArgumentException.class, () -> chooser.select("missing"));
    }

    @Test
    void ignoresStaleExternalDashboardSelections() {
        Action action = Actions.waitSeconds(1.0);
        AutoChooser chooser = Autos.chooser().defaultAuto("Current", action);

        assertEquals(false, chooser.selectIfPresent(null));
        assertEquals(false, chooser.selectIfPresent("auto"));
        assertEquals("Current", chooser.selectedName());
        assertEquals(true, chooser.selectIfPresent(" Current "));
    }

    @Test
    void stopReturnsTheExactRunningAction() {
        Action action = Actions.waitSeconds(1.0);
        AutoChooser chooser = Autos.chooser("Autos").defaultAuto("Score", action);

        chooser.start();

        assertSame(action, chooser.stop().orElseThrow());
        assertTrue(chooser.runningAction().isEmpty());
    }

    @Test
    void namesAutosFromPathOrActionTypeWhenNoLabelIsGiven() {
        Action wait = Actions.waitSeconds(1.0);
        Action path = Actions.sequence().run(new ca.frc6390.athena.mechanism.core.PathAction("choreo", "ScoreTwo"));
        AutoChooser chooser = Autos.chooser()
                .defaultAuto(path)
                .auto(wait);

        assertEquals("ScoreTwo", chooser.defaultName());
        assertTrue(chooser.optionNames().contains(wait.getClass().getSimpleName()));
    }
}
