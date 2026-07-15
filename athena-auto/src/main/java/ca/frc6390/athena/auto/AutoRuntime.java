package ca.frc6390.athena.auto;

import ca.frc6390.athena.mechanism.core.Action;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

/** Selects one named autonomous Action; Athena's mechanism scheduler executes it. */
public final class AutoRuntime {
    private final Map<String, AutoRoutine> routines;
    private String selectedName;
    private Action preparedAction;
    private Action activeAction;
    private long revision;

    public AutoRuntime(Collection<AutoRoutine> routines) {
        this.routines = index(routines);
        selectedName = this.routines.keySet().stream().findFirst().orElse(null);
    }

    public Collection<String> routineNames() { return routines.keySet(); }
    public Optional<AutoRoutine> find(String name) { return Optional.ofNullable(routines.get(normalize(name))); }

    public AutoRuntime select(String name) {
        String normalized = normalize(name);
        if (!routines.containsKey(normalized))
            throw new IllegalArgumentException("Unknown auto routine '" + normalized + "'.");
        selectedName = normalized;
        preparedAction = null;
        activeAction = null;
        revision++;
        return this;
    }

    public String selectedName() {
        if (selectedName == null) throw new IllegalStateException("Auto runtime has no routines.");
        return selectedName;
    }

    public AutoRoutine selectedRoutine() { return routines.get(selectedName()); }

    /** Creates the selected Action once so dashboards can inspect it before autonomous starts. */
    public Action preparedAction() {
        if (preparedAction == null) preparedAction = selectedRoutine().action();
        return preparedAction;
    }

    /** Activates the same selected Action that was prepared for preview. */
    public Action initialize() {
        if (activeAction == null) activeAction = preparedAction();
        return activeAction;
    }

    public Optional<Action> activeAction() { return Optional.ofNullable(activeAction); }
    public boolean active() { return activeAction != null; }
    public long revision() { return revision; }

    /** Ends selection ownership and returns the Action the scheduler must cancel. */
    public Optional<Action> end() {
        Action previous = activeAction;
        if (previous != null) {
            activeAction = null;
            preparedAction = null;
            revision++;
        }
        return Optional.ofNullable(previous);
    }

    public AutoRuntime reset() { activeAction = null; preparedAction = null; revision++; return this; }

    private static Map<String, AutoRoutine> index(Collection<AutoRoutine> routines) {
        Map<String, AutoRoutine> indexed = new LinkedHashMap<>();
        if (routines != null) for (AutoRoutine routine : routines) {
            Objects.requireNonNull(routine, "routine");
            String key = normalize(routine.name());
            if (indexed.putIfAbsent(key, routine) != null)
                throw new IllegalArgumentException("Duplicate auto routine '" + key + "'.");
        }
        return Collections.unmodifiableMap(indexed);
    }

    private static String normalize(String name) {
        return name == null || name.isBlank() ? "auto" : name.trim();
    }
}
