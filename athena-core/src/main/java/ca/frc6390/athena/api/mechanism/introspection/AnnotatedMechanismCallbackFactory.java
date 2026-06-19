package ca.frc6390.athena.api.mechanism.introspection;

import java.lang.reflect.Constructor;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.ParameterizedType;
import java.lang.reflect.Type;
import java.util.Optional;

import ca.frc6390.athena.api.mechanism.StatefulMechanismConfig;
import ca.frc6390.athena.api.mechanism.TypedStatefulMechanismConfig;
import ca.frc6390.athena.api.mechanism.behavior.automation.MechanismStateHookCallback;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismLoopCallback;
import ca.frc6390.athena.api.mechanism.behavior.control.MechanismLoopContext;
import ca.frc6390.athena.core.RobotCore;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.MechanismContext;

final class AnnotatedMechanismCallbackFactory {
    private AnnotatedMechanismCallbackFactory() {
    }

    static Optional<Object> declarationInstance(Class<?> declarationType) {
        if (declarationType == null || declarationType.isInterface() || Modifier.isAbstract(declarationType.getModifiers())) {
            return Optional.empty();
        }
        try {
            Constructor<?> ctor = declarationType.getDeclaredConstructor();
            ctor.setAccessible(true);
            return Optional.of(ctor.newInstance());
        } catch (ReflectiveOperationException ex) {
            return Optional.empty();
        }
    }

    static Optional<Object> fieldValue(Field field, Optional<Object> declaration) {
        try {
            field.setAccessible(true);
            if (Modifier.isStatic(field.getModifiers())) {
                return Optional.ofNullable(field.get(null));
            }
            return declaration.map(instance -> {
                try {
                    return field.get(instance);
                } catch (IllegalAccessException ex) {
                    throw new IllegalStateException("Failed to read field: " + field.getName(), ex);
                }
            });
        } catch (IllegalAccessException ex) {
            throw new IllegalStateException("Failed to read field: " + field.getName(), ex);
        }
    }

    static MechanismLoopCallback loopCallback(Method method, Optional<Object> declaration) {
        Method invokable = prepare(method);
        Object receiver = receiver(invokable, declaration).orElse(null);
        return context -> {
            Object result = invoke(invokable, receiver, loopArguments(invokable, context));
            if (result == null) {
                return 0.0;
            }
            if (result instanceof Number number) {
                return number.doubleValue();
            }
            throw new IllegalStateException("Annotated control loop method must return a numeric value: " + invokable);
        };
    }

    static MechanismStateHookCallback automationCallback(Method method, Optional<Object> declaration) {
        Method invokable = prepare(method);
        Object receiver = receiver(invokable, declaration).orElse(null);
        return context -> invoke(invokable, receiver, automationArguments(invokable, context));
    }

    static Optional<Class<?>> stateType(Class<?> declarationType) {
        return declarationType == null ? Optional.empty() : stateType((Type) declarationType);
    }

    private static Optional<Class<?>> stateType(Type type) {
        if (type == null) {
            return Optional.empty();
        }
        if (type instanceof ParameterizedType parameterized) {
            Type raw = parameterized.getRawType();
            if (raw == TypedStatefulMechanismConfig.class || raw == StatefulMechanismConfig.class) {
                return erase(parameterized.getActualTypeArguments()[0]);
            }
            return stateType(raw);
        }
        if (type instanceof Class<?> rawClass) {
            for (Type iface : rawClass.getGenericInterfaces()) {
                Optional<Class<?>> resolved = stateType(iface);
                if (resolved.isPresent()) {
                    return resolved;
                }
            }
            return stateType(rawClass.getGenericSuperclass());
        }
        return Optional.empty();
    }

    private static Optional<Class<?>> erase(Type type) {
        if (type instanceof Class<?> rawClass) {
            return Optional.of(rawClass);
        }
        if (type instanceof ParameterizedType parameterized && parameterized.getRawType() instanceof Class<?> rawClass) {
            return Optional.of(rawClass);
        }
        return Optional.empty();
    }

    private static Method prepare(Method method) {
        method.setAccessible(true);
        return method;
    }

    private static Optional<Object> receiver(Method method, Optional<Object> declaration) {
        if (Modifier.isStatic(method.getModifiers())) {
            return Optional.empty();
        }
        if (declaration.isPresent()) {
            return declaration;
        }
        throw new IllegalArgumentException(
                "Annotated method requires a declaration instance: " + method.getDeclaringClass().getName() + "#" + method.getName());
    }

    private static Object[] loopArguments(Method method, MechanismLoopContext context) {
        Class<?>[] parameterTypes = method.getParameterTypes();
        Object[] args = new Object[parameterTypes.length];
        for (int i = 0; i < parameterTypes.length; i++) {
            args[i] = resolveLoopArgument(parameterTypes[i], context, method);
        }
        return args;
    }

    private static Object[] automationArguments(Method method, MechanismContext<? extends Mechanism, ?> context) {
        Class<?>[] parameterTypes = method.getParameterTypes();
        Object[] args = new Object[parameterTypes.length];
        for (int i = 0; i < parameterTypes.length; i++) {
            args[i] = resolveAutomationArgument(parameterTypes[i], context, method);
        }
        return args;
    }

    private static Object resolveLoopArgument(Class<?> type, MechanismLoopContext context, Method method) {
        if (MechanismLoopContext.class.isAssignableFrom(type)) {
            return context;
        }
        if (RobotCore.class.isAssignableFrom(type)) {
            return context.robotCore();
        }
        if (Mechanism.class.isAssignableFrom(type)) {
            return context.mechanism();
        }
        Object state = context.state();
        if (state != null && type.isInstance(state)) {
            return state;
        }
        throw new IllegalArgumentException("Unsupported annotated control loop parameter " + type.getName() + " for " + method);
    }

    private static Object resolveAutomationArgument(
            Class<?> type,
            MechanismContext<? extends Mechanism, ?> context,
            Method method) {
        if (MechanismContext.class.isAssignableFrom(type)) {
            return context;
        }
        if (RobotCore.class.isAssignableFrom(type)) {
            return context.robotCore();
        }
        if (Mechanism.class.isAssignableFrom(type)) {
            return context.mechanism();
        }
        Object state = context.state();
        if (state != null && type.isInstance(state)) {
            return state;
        }
        throw new IllegalArgumentException("Unsupported annotated automation parameter " + type.getName() + " for " + method);
    }

    private static Object invoke(Method method, Object receiver, Object[] args) {
        try {
            return method.invoke(receiver, args);
        } catch (IllegalAccessException ex) {
            throw new IllegalStateException("Failed to invoke annotated method: " + method, ex);
        } catch (InvocationTargetException ex) {
            Throwable cause = ex.getCause() != null ? ex.getCause() : ex;
            if (cause instanceof RuntimeException runtime) {
                throw runtime;
            }
            throw new IllegalStateException("Annotated method threw: " + method, cause);
        }
    }
}
