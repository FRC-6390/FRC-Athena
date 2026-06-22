package ca.frc6390.athena.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import org.junit.jupiter.api.Test;

import ca.frc6390.athena.commands.CommandRunner;
import ca.frc6390.athena.commands.CommandSpec;

class AutoChooserConfigTest {
    @Test
    void lowersAndSelectsDefaultRoutine() {
        AutoChooserSpec spec = Autos.chooser()
                .routine("leave", routine -> routine
                        .displayName("Leave Community")
                        .command(CommandSpec.create("leave").toSpec()))
                .routine("score", routine -> routine.command(CommandSpec.create("score").toSpec()))
                .defaultRoutine("score")
                .toSpec();

        assertFalse(spec.validate().hasErrors());
        assertEquals("score", spec.prepare().selectedRoutine().id());
        assertEquals("score", spec.prepare().selectedCommand().name());
    }

    @Test
    void prepareFailsWithoutRegisteredRoutine() {
        AutoChooserSpec spec = Autos.chooser().toSpec();

        assertTrue(spec.validate().hasErrors());
        assertFalse(spec.validate().errorsWithCode("auto.no-routines").isEmpty());
        assertThrows(RuntimeException.class, spec::prepare);
    }

    @Test
    void defaultMustReferenceRegisteredRoutine() {
        AutoChooserSpec spec = Autos.chooser()
                .routine("leave", routine -> routine.command(CommandSpec.create("leave").toSpec()))
                .defaultRoutine("missing")
                .toSpec();

        assertTrue(spec.validate().hasErrors());
        assertFalse(spec.validate().errorsWithCode("auto.default-missing").isEmpty());
    }

    @Test
    void duplicateRoutineIdsAreReported() {
        AutoChooserSpec spec = Autos.chooser()
                .routine("leave", routine -> routine.command(CommandSpec.create("leaveA").toSpec()))
                .routine("leave", routine -> routine.command(CommandSpec.create("leaveB").toSpec()))
                .toSpec();

        assertTrue(spec.validate().hasErrors());
        assertFalse(spec.validate().errorsWithCode("auto.duplicate-routine").isEmpty());
    }

    @Test
    void executionCanSwitchSelectedRoutineAndRunCommand() {
        AtomicInteger cycles = new AtomicInteger();
        AutoChooserSpec spec = Autos.chooser()
                .routine("leave", routine -> routine.command(CommandSpec.create("leave").toSpec()))
                .routine("score", routine -> routine.command(CommandSpec.create("score")
                        .onExecute(cycles::incrementAndGet)
                        .until(() -> cycles.get() >= 2)
                        .toSpec()))
                .toSpec();

        AutoExecution execution = spec.prepare().select("score");
        CommandRunner runner = new CommandRunner(execution.selectedCommand());

        runner.step();
        runner.step();

        assertEquals("score", execution.selectedRoutine().id());
        assertEquals(2, cycles.get());
    }

    @Test
    void routineCanLoadFromRegisteredSource() {
        AutoRegistry.get().clear();
        try {
            AutoRegistry.get().register("sim", path -> CommandSpec.create("sim:" + path).toSpec());

            AutoChooserSpec spec = Autos.chooser()
                    .routine("leave", routine -> routine.fromSource("sim", "leave-path"))
                    .toSpec();

            assertEquals("sim:leave-path", spec.prepare().selectedCommand().name());
        } finally {
            AutoRegistry.get().clear();
        }
    }

    @Test
    void executionCarriesScopedInputsBetweenRoutines() {
        AutoChooserSpec spec = Autos.chooser()
                .routine("producer", routine -> routine.command(CommandSpec.create("producer").toSpec()))
                .routine("consumer", routine -> routine.command(CommandSpec.create("consumer").toSpec()))
                .toSpec();

        AutoExecution execution = spec.prepare();
        execution.inputs().scope("consumer")
                .string("targetMode", "amp")
                .bool("fire", true)
                .number("delaySeconds", 0.35);

        execution.select("consumer");

        assertEquals("amp", execution.selectedInputs().readString("targetMode", "speaker"));
        assertEquals(true, execution.selectedInputs().readBool("fire", false));
        assertEquals(0.35, execution.selectedInputs().readNumber("delaySeconds", 0.0), 1.0e-9);
    }

    @Test
    void executionInputsSupportSuppliersAndScopedClear() {
        AtomicReference<String> targetMode = new AtomicReference<>("speaker");
        AutoChooserSpec spec = Autos.chooser()
                .routine("consumer", routine -> routine.command(CommandSpec.create("consumer").toSpec()))
                .toSpec();

        AutoExecution execution = spec.prepare();
        execution.inputs().scope("consumer")
                .stringSupplier("targetMode", targetMode::get)
                .bool("fire", true);

        targetMode.set("amp");
        assertEquals("amp", execution.selectedInputs().readString("targetMode", "speaker"));
        assertEquals(true, execution.selectedInputs().readBool("fire", false));

        execution.inputs().clearScope("consumer");
        assertEquals("speaker", execution.selectedInputs().readString("targetMode", "speaker"));
        assertEquals(false, execution.selectedInputs().readBool("fire", false));
    }
}
