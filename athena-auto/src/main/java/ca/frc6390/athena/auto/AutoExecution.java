package ca.frc6390.athena.auto;

import ca.frc6390.athena.commands.CommandSpec;

/**
 * Prepared autonomous execution view.
 */
public final class AutoExecution {
    private final AutoChooserSpec chooser;
    private final AutoInputStore inputs = new AutoInputStore();
    private String selectedRoutineId;

    AutoExecution(AutoChooserSpec chooser, String selectedRoutineId) {
        this.chooser = chooser;
        this.selectedRoutineId = selectedRoutineId;
    }

    /**
     * Selects a registered routine.
     *
     * @param routineId routine id
     * @return this execution view
     */
    public AutoExecution select(String routineId) {
        if (chooser.findRoutine(routineId).isEmpty()) {
            throw new IllegalArgumentException("Unknown auto routine '" + routineId + "'.");
        }
        selectedRoutineId = routineId;
        return this;
    }

    /**
     * Returns selected routine metadata.
     *
     * @return selected routine
     */
    public AutoRoutineSpec selectedRoutine() {
        return chooser.findRoutine(selectedRoutineId).orElseThrow();
    }

    /**
     * Returns selected command descriptor.
     *
     * @return selected command
     */
    public CommandSpec selectedCommand() {
        return selectedRoutine().command();
    }

    /**
     * Returns scoped autonomous inputs for this execution.
     *
     * @return input store
     */
    public AutoInputStore inputs() {
        return inputs;
    }

    /**
     * Returns inputs scoped to the selected routine.
     *
     * @return selected routine input scope
     */
    public AutoInputScope selectedInputs() {
        return inputs.scope(selectedRoutineId);
    }
}
