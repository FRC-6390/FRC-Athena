package ca.frc6390.athena.auto;

import ca.frc6390.athena.commands.CommandState;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

/**
 * Selects and owns the active autonomous command state.
 */
public final class AutoRuntime {
    private final Map<String, AutoRoutine> routines;
    private String selectedName;
    private CommandState activeState;
    private boolean activeInitialized;

    /**
     * Creates an auto runtime.
     *
     * @param routines available routines
     */
    public AutoRuntime(Collection<AutoRoutine> routines) {
        this.routines = index(routines);
        selectedName = this.routines.keySet().stream().findFirst().orElse(null);
    }

    /**
     * Returns the available routine names in registration order.
     *
     * @return routine names
     */
    public Collection<String> routineNames() {
        return routines.keySet();
    }

    /**
     * Finds a routine by name.
     *
     * @param name routine name
     * @return routine, if present
     */
    public Optional<AutoRoutine> find(String name) {
        return Optional.ofNullable(routines.get(normalize(name)));
    }

    /**
     * Selects a routine by name.
     *
     * @param name routine name
     * @return this runtime
     */
    public AutoRuntime select(String name) {
        String normalized = normalize(name);
        if (!routines.containsKey(normalized)) {
            throw new IllegalArgumentException("Unknown auto routine '" + normalized + "'.");
        }
        end(true);
        selectedName = normalized;
        activeState = null;
        activeInitialized = false;
        return this;
    }

    /**
     * Returns the selected routine name.
     *
     * @return selected name
     */
    public String selectedName() {
        if (selectedName == null) {
            throw new IllegalStateException("Auto runtime has no routines.");
        }
        return selectedName;
    }

    /**
     * Returns the selected routine.
     *
     * @return selected routine
     */
    public AutoRoutine selectedRoutine() {
        if (selectedName == null) {
            throw new IllegalStateException("Auto runtime has no routines.");
        }
        return routines.get(selectedName);
    }

    /**
     * Returns the selected command state, creating it lazily once per selection.
     *
     * @return selected command state
     */
    public CommandState selectedState() {
        if (activeState == null) {
            activeState = selectedRoutine().state();
        }
        return activeState;
    }

    /**
     * Initializes the selected autonomous command state once.
     *
     * @return this runtime
     */
    public AutoRuntime initialize() {
        if (!activeInitialized) {
            selectedState().onInitialize().run();
            activeInitialized = true;
        }
        return this;
    }

    /**
     * Executes one cycle of the selected autonomous command state.
     *
     * @return this runtime
     */
    public AutoRuntime execute() {
        initialize();
        selectedState().onExecute().run();
        return this;
    }

    /**
     * Returns whether the selected autonomous command state is finished.
     *
     * @return true when finished
     */
    public boolean isFinished() {
        return selectedState().isFinished().getAsBoolean();
    }

    /**
     * Ends the selected autonomous command state if it has been initialized.
     *
     * @param interrupted true when the runtime is ending because of interruption
     * @return this runtime
     */
    public AutoRuntime end(boolean interrupted) {
        if (activeInitialized && activeState != null) {
            activeState.onEnd().run();
        }
        activeInitialized = false;
        return this;
    }

    /**
     * Executes one cycle and ends the command state when its finish condition is met.
     *
     * @return true when the command state finished during or before this cycle
     */
    public boolean periodic() {
        execute();
        if (isFinished()) {
            end(false);
            return true;
        }
        return false;
    }

    /**
     * Clears the active command state so the selected routine can create a fresh one.
     *
     * @return this runtime
     */
    public AutoRuntime reset() {
        end(true);
        activeState = null;
        activeInitialized = false;
        return this;
    }

    private static Map<String, AutoRoutine> index(Collection<AutoRoutine> routines) {
        Map<String, AutoRoutine> indexed = new LinkedHashMap<>();
        if (routines == null) {
            return indexed;
        }
        for (AutoRoutine routine : routines) {
            Objects.requireNonNull(routine, "routine");
            String key = normalize(routine.name());
            AutoRoutine previous = indexed.putIfAbsent(key, routine);
            if (previous != null) {
                throw new IllegalArgumentException("Duplicate auto routine '" + key + "'.");
            }
        }
        return Collections.unmodifiableMap(indexed);
    }

    private static String normalize(String name) {
        return name == null || name.isBlank() ? "auto" : name.trim();
    }
}
