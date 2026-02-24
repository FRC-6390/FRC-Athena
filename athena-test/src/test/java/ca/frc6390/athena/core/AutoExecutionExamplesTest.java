package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertNotNull;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.core.examples.AutoExecutionExamples;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.ArrayList;
import org.junit.jupiter.api.Test;

final class AutoExecutionExamplesTest {

    @Test
    void programChooserSelectsDefaultProgram() {
        RobotAuto autos = AutoExecutionExamples.createWithTwoCustomPrograms(new ArrayList<>());

        AutoExecutionExamples.configureProgramChooser(autos, "safe");

        assertEquals("safe", autos.selection().selectedProgram().orElseThrow().key().id());
        Command selected = AutoExecutionExamples.selectedCommand(autos).orElseThrow();
        assertNotNull(selected);
    }

    @Test
    void commandChooserSelectionMapsToProgramAndCommand() {
        RobotAuto autos = AutoExecutionExamples.createWithTwoCustomPrograms(new ArrayList<>());

        AutoExecutionExamples.configureCommandChooser(autos, "score");

        Command selected = AutoExecutionExamples.selectedCommand(autos).orElseThrow();
        assertEquals(autos.selection().commandChooser().orElseThrow().getSelected(), selected);
        assertEquals("score", autos.selection().selectedProgram().orElseThrow().key().id());
    }

    @Test
    void chooserRequiresRegisteredAutos() {
        RobotAuto autos = new RobotAuto();
        assertThrows(IllegalStateException.class,
                () -> autos.selection().programChooser(RobotAuto.AutoKey.of("none")));
    }

    @Test
    void chooserRequiresValidDefaultId() {
        RobotAuto autos = AutoExecutionExamples.createWithTwoCustomPrograms(new ArrayList<>());
        assertThrows(IllegalArgumentException.class,
                () -> autos.selection().programChooser(RobotAuto.AutoKey.of("missing")));
    }

    @Test
    void executionPrepareFinalizesWithoutThrowing() {
        RobotAuto autos = AutoExecutionExamples.createWithTwoCustomPrograms(new ArrayList<>());
        autos.execution().prepare();
        assertTrue(autos.registry().hasRoutine("safe"));
        assertTrue(autos.registry().hasRoutine("score"));
    }
}
