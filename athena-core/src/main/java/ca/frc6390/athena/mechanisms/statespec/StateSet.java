package ca.frc6390.athena.mechanisms.statespec;

import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.Map;
import java.lang.StackWalker;
import java.lang.StackWalker.StackFrame;

/**
 * Named collection of source-visible state keys.
 */
public class StateSet {
    private static final StackWalker CALLER_WALKER = StackWalker.getInstance(StackWalker.Option.RETAIN_CLASS_REFERENCE);
    private static final ClassValue<StateSet> REGISTRIES = new ClassValue<>() {
        @Override
        protected StateSet computeValue(Class<?> type) {
            return new StateSet(type, true);
        }
    };

    private final Map<String, StateId> states = new LinkedHashMap<>();
    private final Class<?> ownerType;
    private final boolean registry;

    public StateSet() {
        this.ownerType = getClass();
        this.registry = false;
    }

    private StateSet(Class<?> ownerType, boolean registry) {
        this.ownerType = ownerType;
        this.registry = registry;
    }

    protected static StateId state(String name) {
        return callerRegistry().stateInternal(name);
    }

    protected static StateId state(String name, double setpoint) {
        return state(name).setpoint(setpoint);
    }

    protected static StateId state(String name, Object setpoint) {
        return state(name).setpoint(setpoint);
    }

    private StateId stateInternal(String name) {
        if (states.containsKey(name)) {
            throw new IllegalArgumentException("duplicate state name: " + name);
        }
        StateId state = new StateId(this, name);
        states.put(name, state);
        return state;
    }

    public final Class<?> ownerType() {
        return registry().ownerType;
    }

    public final StateId get(String name) {
        return registry().states.get(name);
    }

    public final StateId require(String name) {
        StateId state = get(name);
        if (state == null) {
            throw new IllegalArgumentException("unknown state: " + name);
        }
        return state;
    }

    public final Collection<StateId> states() {
        return java.util.List.copyOf(registry().states.values());
    }

    private StateSet registry() {
        return registry ? this : REGISTRIES.get(ownerType);
    }

    private static StateSet callerRegistry() {
        return REGISTRIES.get(callerType());
    }

    private static Class<?> callerType() {
        return CALLER_WALKER.walk(frames -> frames
            .map(StackFrame::getDeclaringClass)
            .filter(type -> type != StateSet.class)
            .findFirst()
            .orElse(StateSet.class));
    }
}
