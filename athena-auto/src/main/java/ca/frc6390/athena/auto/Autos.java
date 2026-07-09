package ca.frc6390.athena.auto;

import ca.frc6390.athena.commands.CommandAction;
import java.util.Arrays;
import java.util.Collection;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

/**
 * Catalog for autonomous routines and runtimes.
 */
public final class Autos {
    private Autos() {
    }

    /**
     * Creates a named routine from a command Action.
     *
     * @param name routine name
     * @param Action command Action
     * @return auto routine
     */
    public static AutoRoutine routine(String name, CommandAction Action) {
        Objects.requireNonNull(Action, "Action");
        return routine(name, () -> Action);
    }

    /**
     * Creates a named routine from a command Action and marker bindings.
     *
     * @param name routine name
     * @param Action command Action
     * @param markers path marker bindings
     * @return auto routine
     */
    public static AutoRoutine routine(String name, CommandAction Action, PathMarkerBinding... markers) {
        Objects.requireNonNull(Action, "Action");
        return routine(name, () -> Action, markers);
    }

    /**
     * Creates a named routine from a command Action factory.
     *
     * @param name routine name
     * @param stateFactory command Action factory
     * @return auto routine
     */
    public static AutoRoutine routine(String name, Supplier<CommandAction> stateFactory) {
        return new AutoRoutine(name, stateFactory);
    }

    /**
     * Creates a named routine from a command Action factory and marker bindings.
     *
     * @param name routine name
     * @param stateFactory command Action factory
     * @param markers path marker bindings
     * @return auto routine
     */
    public static AutoRoutine routine(String name, Supplier<CommandAction> stateFactory, PathMarkerBinding... markers) {
        return new AutoRoutine(name, stateFactory, markers == null ? List.of() : Arrays.asList(markers));
    }

    /**
     * Creates a path marker binding.
     *
     * @param marker marker name
     * @param Action command Action
     * @return marker binding
     */
    public static PathMarkerBinding marker(String marker, CommandAction Action) {
        return new PathMarkerBinding(marker, Action);
    }

    /**
     * Creates a named routine from a path provider.
     *
     * @param name routine name
     * @param provider path provider
     * @param pathName provider path name
     * @return auto routine
     */
    public static AutoRoutine path(String name, PathProvider provider, String pathName) {
        return path(name, provider, pathName, List.of());
    }

    /**
     * Creates a named routine from a path provider and marker bindings.
     *
     * @param name routine name
     * @param provider path provider
     * @param pathName provider path name
     * @param markers path marker bindings
     * @return auto routine
     */
    public static AutoRoutine path(String name, PathProvider provider, String pathName, PathMarkerBinding... markers) {
        return path(name, provider, pathName, markers == null ? List.of() : Arrays.asList(markers));
    }

    /**
     * Creates a named routine from a path provider and marker bindings.
     *
     * @param name routine name
     * @param provider path provider
     * @param pathName provider path name
     * @param markers path marker bindings
     * @return auto routine
     */
    public static AutoRoutine path(String name, PathProvider provider, String pathName, Collection<PathMarkerBinding> markers) {
        Objects.requireNonNull(provider, "provider");
        return new AutoRoutine(name, () -> provider.load(pathName), List.copyOf(markers));
    }

    /**
     * Creates an auto runtime.
     *
     * @param routines routines
     * @return auto runtime
     */
    public static AutoRuntime runtime(AutoRoutine... routines) {
        return runtime(routines == null ? List.of() : Arrays.asList(routines));
    }

    /**
     * Creates an auto runtime.
     *
     * @param routines routines
     * @return auto runtime
     */
    public static AutoRuntime runtime(Collection<AutoRoutine> routines) {
        return new AutoRuntime(routines);
    }
}
