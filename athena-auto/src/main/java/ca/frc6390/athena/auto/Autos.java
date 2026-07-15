package ca.frc6390.athena.auto;

import ca.frc6390.athena.mechanism.core.Action;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

/** Factories for naming ordinary Actions as selectable autonomous routines. */
public final class Autos {
    private Autos() { }

    public static AutoRoutine routine(String name, Action action) {
        return new AutoRoutine(name, Objects.requireNonNull(action, "action"));
    }

    public static AutoRoutine routine(String name, Supplier<Action> actionFactory) {
        return new AutoRoutine(name, actionFactory);
    }

    public static AutoRoutine path(String name, PathProvider provider, String pathName) {
        Objects.requireNonNull(provider, "provider");
        return routine(name, provider.path(pathName));
    }

    public static AutoRuntime runtime(AutoRoutine... routines) {
        return runtime(routines == null ? List.of() : Arrays.asList(routines));
    }

    public static AutoRuntime runtime(Collection<AutoRoutine> routines) {
        return new AutoRuntime(routines);
    }
}
