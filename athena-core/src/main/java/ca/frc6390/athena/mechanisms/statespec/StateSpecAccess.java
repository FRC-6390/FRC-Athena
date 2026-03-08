package ca.frc6390.athena.mechanisms.statespec;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Runtime bridge for state enums that are augmented by Athena's javac plugin.
 *
 * <p>JDT does not reliably see the injected interfaces in-editor, so Athena's public API
 * cannot depend on those bounds for IDE correctness. At runtime the generated enum methods
 * still exist, and this helper resolves them via the normal interfaces first and reflection
 * as a fallback.</p>
 */
public final class StateSpecAccess {
    private static final Map<Class<?>, Method> SETPOINT_METHODS = new ConcurrentHashMap<>();
    private static final Map<Class<?>, Method> SEED_METHODS = new ConcurrentHashMap<>();
    private static final Method MISSING_METHOD = missingMethod();

    private StateSpecAccess() {}

    @SuppressWarnings("unchecked")
    public static <T> T setpoint(Enum<?> state) {
        if (state == null) {
            return null;
        }
        if (state instanceof SetpointProvider<?> provider) {
            return (T) provider.getSetpoint();
        }

        Object value = invokeGenerated(state, "getSetpoint", SETPOINT_METHODS, true);
        return (T) value;
    }

    @SuppressWarnings("unchecked")
    public static <E extends Enum<E>> StateSeed<E> seed(E state) {
        if (state == null) {
            return null;
        }
        if (state instanceof StateSeedProvider<?> provider) {
            return (StateSeed<E>) provider.seed();
        }

        Object value = invokeGenerated(state, "seed", SEED_METHODS, false);
        if (value == null) {
            return null;
        }
        if (!(value instanceof StateSeed<?> seed)) {
            throw new IllegalStateException("Generated seed() on " + state.getClass().getName()
                    + " returned " + value.getClass().getName() + " instead of StateSeed.");
        }
        return (StateSeed<E>) seed;
    }

    @SuppressWarnings("unchecked")
    public static <E extends Enum<E>> E resolve(E context, String stateName) {
        if (context == null || stateName == null || stateName.isBlank()) {
            return null;
        }
        Class<?> declaringClass = context.getDeclaringClass();
        if (!Enum.class.isAssignableFrom(declaringClass)) {
            return null;
        }
        return Enum.valueOf((Class<E>) declaringClass.asSubclass(Enum.class), stateName);
    }

    private static Object invokeGenerated(
            Enum<?> state,
            String methodName,
            Map<Class<?>, Method> cache,
            boolean required) {
        Method method = cache.computeIfAbsent(state.getClass(), type -> findMethod(type, methodName));
        if (method == MISSING_METHOD) {
            if (required) {
                throw missingContract(state, methodName);
            }
            return null;
        }

        try {
            return method.invoke(state);
        } catch (IllegalAccessException | InvocationTargetException ex) {
            throw new IllegalStateException(
                    "Could not invoke generated " + methodName + "() on " + state.getClass().getName(),
                    ex);
        }
    }

    private static Method findMethod(Class<?> type, String methodName) {
        Objects.requireNonNull(type, "type");
        try {
            Method method = type.getMethod(methodName);
            method.setAccessible(true);
            return method;
        } catch (NoSuchMethodException ex) {
            return MISSING_METHOD;
        }
    }

    private static IllegalStateException missingContract(Enum<?> state, String methodName) {
        return new IllegalStateException(
                "State enum " + state.getClass().getName() + " is missing generated " + methodName
                        + "(). Ensure Athena DSL/plugin support is enabled for this project.");
    }

    private static Method missingMethod() {
        try {
            return StateSpecAccess.class.getDeclaredMethod("sentinel");
        } catch (NoSuchMethodException ex) {
            throw new IllegalStateException("StateSpecAccess sentinel method missing", ex);
        }
    }

    @SuppressWarnings("unused")
    private static void sentinel() {}
}
