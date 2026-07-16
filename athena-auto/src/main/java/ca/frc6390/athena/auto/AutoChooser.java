package ca.frc6390.athena.auto;

import ca.frc6390.athena.mechanism.core.Action;
import java.util.Collection;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;

/**
 * A named collection of ordinary Actions selectable for autonomous.
 * Selection is inert; Athena freezes and starts the selected Action only when autonomous begins.
 */
public final class AutoChooser {
    private final String dashboardName;
    private final Map<String, Action> options = new LinkedHashMap<>();
    private String defaultName;
    private String selectedName;
    private String runningName;
    private Action runningAction;
    private long revision;

    AutoChooser(String dashboardName) {
        this.dashboardName = normalize(dashboardName, "Auto Chooser");
    }

    /** Adds and selects the default autonomous Action. */
    public AutoChooser defaultAuto(String name, Action action) {
        String normalized = add(name, action);
        defaultName = normalized;
        if (selectedName == null) selectedName = normalized;
        return this;
    }

    /** Adds a default auto named from the Action type. */
    public AutoChooser defaultAuto(Action action) {
        return defaultAuto(actionName(action), action);
    }

    /** Adds another autonomous Action. */
    public AutoChooser auto(String name, Action action) {
        String normalized = add(name, action);
        if (selectedName == null) selectedName = normalized;
        return this;
    }

    /** Adds an auto named from the Action type. */
    public AutoChooser auto(Action action) {
        return auto(actionName(action), action);
    }

    /** Changes the next autonomous selection without starting or interrupting any Action. */
    public AutoChooser select(String name) {
        String normalized = normalize(name, "auto");
        if (!options.containsKey(normalized)) {
            throw new IllegalArgumentException("Unknown auto '" + normalized + "'.");
        }
        if (!Objects.equals(selectedName, normalized)) {
            selectedName = normalized;
            revision++;
        }
        return this;
    }

    /**
     * Applies an external dashboard selection only when it is a current option.
     * Stale persisted dashboard values are ignored instead of stopping the robot program.
     */
    public boolean selectIfPresent(String name) {
        if (name == null || name.isBlank()) return false;
        String normalized = name.trim();
        if (!options.containsKey(normalized)) return false;
        select(normalized);
        return true;
    }

    public String dashboardName() {
        return dashboardName;
    }

    public Collection<String> optionNames() {
        return Collections.unmodifiableSet(options.keySet());
    }

    public Map<String, Action> options() {
        return Collections.unmodifiableMap(options);
    }

    public String defaultName() {
        requireOptions();
        return defaultName == null ? options.keySet().iterator().next() : defaultName;
    }

    public String selectedName() {
        requireOptions();
        return selectedName;
    }

    public Action selectedAction() {
        return options.get(selectedName());
    }

    public Optional<String> runningName() {
        return Optional.ofNullable(runningName);
    }

    public Optional<Action> runningAction() {
        return Optional.ofNullable(runningAction);
    }

    public long revision() {
        return revision;
    }

    /** Freezes the current selection for this autonomous period. */
    public Action start() {
        requireOptions();
        if (runningAction == null) {
            runningName = selectedName;
            runningAction = selectedAction();
        }
        return runningAction;
    }

    /** Releases lifecycle ownership of the running Action. */
    public Optional<Action> stop() {
        Action previous = runningAction;
        runningAction = null;
        runningName = null;
        return Optional.ofNullable(previous);
    }

    private String add(String name, Action action) {
        String normalized = normalize(name, "auto");
        if (options.putIfAbsent(normalized, Objects.requireNonNull(action, "action")) != null) {
            throw new IllegalArgumentException("Duplicate auto '" + normalized + "'.");
        }
        revision++;
        return normalized;
    }

    private void requireOptions() {
        if (options.isEmpty()) throw new IllegalStateException("Auto chooser has no autos.");
    }

    private static String normalize(String value, String fallback) {
        return value == null || value.isBlank() ? fallback : value.trim();
    }

    private static String actionName(Action action) {
        Action safeAction = Objects.requireNonNull(action, "action");
        var paths = AutoPlan.inspect(safeAction).paths();
        if (!paths.isEmpty()) {
            return normalize(paths.get(0).name(), paths.get(0).key());
        }
        String simpleName = safeAction.getClass().getSimpleName();
        return normalize(simpleName, safeAction.getClass().getName());
    }
}
