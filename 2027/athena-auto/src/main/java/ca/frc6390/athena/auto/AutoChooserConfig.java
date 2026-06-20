package ca.frc6390.athena.auto;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

/**
 * Student-facing autonomous chooser declaration.
 */
public final class AutoChooserConfig {
    private final List<AutoRoutineConfig> routines = new ArrayList<>();
    private String defaultRoutineId;

    /**
     * Adds an autonomous routine.
     *
     * @param id stable routine id
     * @param configure routine configuration
     * @return this config
     */
    public AutoChooserConfig routine(String id, Consumer<AutoRoutineConfig> configure) {
        AutoRoutineConfig routine = new AutoRoutineConfig(id);
        if (configure != null) {
            configure.accept(routine);
        }
        routines.add(routine);
        return this;
    }

    /**
     * Sets the default routine id.
     *
     * @param routineId default routine id
     * @return this config
     */
    public AutoChooserConfig defaultRoutine(String routineId) {
        defaultRoutineId = routineId;
        return this;
    }

    /**
     * Lowers this declaration to an immutable chooser spec.
     *
     * @return chooser spec
     */
    public AutoChooserSpec toSpec() {
        List<AutoRoutineSpec> specs = routines.stream()
                .map(AutoRoutineConfig::toSpec)
                .toList();
        return new AutoChooserSpec(specs, defaultRoutineId);
    }
}
