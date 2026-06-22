package ca.frc6390.athena.auto;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import ca.frc6390.athena.runtime.validation.ValidationReport;

/**
 * Immutable autonomous chooser declaration.
 *
 * @param routines registered routines
 * @param defaultRoutineId default routine id
 */
public record AutoChooserSpec(List<AutoRoutineSpec> routines, String defaultRoutineId) {
    public AutoChooserSpec {
        routines = routines == null ? List.of() : List.copyOf(routines);
        if ((defaultRoutineId == null || defaultRoutineId.isBlank()) && !routines.isEmpty()) {
            defaultRoutineId = routines.get(0).id();
        }
    }

    /**
     * Validates routine registration and default selection.
     *
     * @return validation report
     */
    public ValidationReport validate() {
        ValidationReport.Builder report = ValidationReport.builder();
        if (routines.isEmpty()) {
            report.error("auto.no-routines", "auto", "Auto chooser needs at least one routine.");
            return report.build();
        }
        Set<String> ids = new HashSet<>();
        for (AutoRoutineSpec routine : routines) {
            if (!ids.add(routine.id())) {
                report.error("auto.duplicate-routine", routine.id(), "Auto routine ids must be unique.");
            }
        }
        if (!ids.contains(defaultRoutineId)) {
            report.error("auto.default-missing", defaultRoutineId, "Default auto routine must reference a registered routine.");
        }
        return report.build();
    }

    /**
     * Finds a routine by id.
     *
     * @param id routine id
     * @return routine if registered
     */
    public Optional<AutoRoutineSpec> findRoutine(String id) {
        return routines.stream()
                .filter(routine -> routine.id().equals(id))
                .findFirst();
    }

    /**
     * Prepares execution with the default routine selected.
     *
     * @return execution view
     */
    public AutoExecution prepare() {
        validate().assertValid();
        return new AutoExecution(this, defaultRoutineId);
    }
}
