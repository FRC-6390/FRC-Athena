package ca.frc6390.athena.examples;

import ca.frc6390.athena.auto.AutoChooserSpec;
import ca.frc6390.athena.auto.AutoExecution;
import ca.frc6390.athena.auto.AutoRegistry;
import ca.frc6390.athena.auto.Autos;
import ca.frc6390.athena.commands.CommandSpec;

/**
 * Autonomous chooser example using generic command descriptors.
 */
public final class AutoExample {
    /** Target mode input key used by auto handoff examples. */
    public static final String TARGET_MODE_KEY = "handoff.targetMode";

    /** Fire trigger input key used by auto handoff examples. */
    public static final String FIRE_TRIGGER_KEY = "handoff.fireTrigger";

    private AutoExample() {
    }

    /**
     * Creates an auto chooser with explicit command descriptors.
     *
     * @return auto chooser spec
     */
    public static AutoChooserSpec chooser() {
        return Autos.chooser()
                .routine("leave", routine -> routine
                        .displayName("Leave Community")
                        .command(CommandSpec.create("leave").toSpec()))
                .routine("score", routine -> routine
                        .displayName("Score Preload")
                        .command(CommandSpec.create("score").toSpec()))
                .defaultRoutine("leave")
                .toSpec();
    }

    /**
     * Creates a chooser that loads a command through a registered source.
     *
     * @return auto chooser spec
     */
    public static AutoChooserSpec sourcedChooser() {
        AutoRegistry.get().register("sim", path -> CommandSpec.create("sim:" + path).toSpec());
        return Autos.chooser()
                .routine("leave", routine -> routine.fromSource("sim", "leave-path"))
                .toSpec();
    }

    /**
     * Creates producer/consumer auto routines that exchange scoped inputs.
     *
     * @return auto chooser spec
     */
    public static AutoChooserSpec handoffChooser() {
        return Autos.chooser()
                .routine("producer", routine -> routine
                        .displayName("Set Consumer Inputs")
                        .command(CommandSpec.create("producer").toSpec()))
                .routine("consumer", routine -> routine
                        .displayName("Use Consumer Inputs")
                        .command(CommandSpec.create("consumer").toSpec()))
                .defaultRoutine("producer")
                .toSpec();
    }

    /**
     * Writes values scoped to the consumer routine.
     *
     * @param execution prepared auto execution
     * @return same execution for chaining
     */
    public static AutoExecution handoffToConsumer(AutoExecution execution) {
        execution.inputs().scope("consumer")
                .string(TARGET_MODE_KEY, "amp")
                .bool(FIRE_TRIGGER_KEY, true)
                .number("handoff.delaySeconds", 0.25);
        return execution;
    }

    /**
     * Reads the target mode for the selected routine.
     *
     * @param execution prepared auto execution
     * @return selected target mode
     */
    public static String selectedTargetMode(AutoExecution execution) {
        return execution.selectedInputs().readString(TARGET_MODE_KEY, "speaker");
    }

    /**
     * Reads whether the selected routine should fire.
     *
     * @param execution prepared auto execution
     * @return true when fire trigger is set
     */
    public static boolean selectedFireTrigger(AutoExecution execution) {
        return execution.selectedInputs().readBool(FIRE_TRIGGER_KEY, false);
    }
}
