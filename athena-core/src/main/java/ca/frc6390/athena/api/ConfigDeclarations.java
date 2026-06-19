package ca.frc6390.athena.api;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationHandler;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.reflect.Proxy;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodType;
import java.util.Objects;

/**
 * Utilities for turning declaration types into lightweight declaration instances.
 */
public final class ConfigDeclarations {
    private static final ClassValue<Object> INSTANCES = new ClassValue<>() {
        @Override
        protected Object computeValue(Class<?> type) {
            return create(type);
        }
    };

    private ConfigDeclarations() {
    }

    public static <T> T instance(Class<T> type) {
        Objects.requireNonNull(type, "type");
        return type.cast(INSTANCES.get(type));
    }

    private static Object create(Class<?> type) {
        if (type.isInterface()) {
            return Proxy.newProxyInstance(
                type.getClassLoader(),
                new Class<?>[] { type },
                new DefaultMethodInvocationHandler(type));
        }
        if (Modifier.isAbstract(type.getModifiers())) {
            throw new IllegalArgumentException(
                "Config declaration type must be an interface or concrete class: " + type.getName());
        }
        try {
            Constructor<?> ctor = type.getDeclaredConstructor();
            ctor.setAccessible(true);
            return ctor.newInstance();
        } catch (ReflectiveOperationException ex) {
            throw new IllegalArgumentException(
                "Config declaration class needs a no-arg constructor: " + type.getName(), ex);
        }
    }

    private record DefaultMethodInvocationHandler(Class<?> declarationType) implements InvocationHandler {
        @Override
        public Object invoke(Object proxy, Method method, Object[] args) throws Throwable {
            if (method.getDeclaringClass() == Object.class) {
                return invokeObject(proxy, method, args);
            }
            if (method.isDefault()) {
                return invokeDefault(proxy, method, args);
            }
            throw new IllegalArgumentException(
                "Config declaration method must be default when using "
                    + declarationType.getSimpleName()
                    + ".class: "
                    + method.getDeclaringClass().getName()
                    + "#"
                    + method.getName());
        }

        private Object invokeObject(Object proxy, Method method, Object[] args) {
            return switch (method.getName()) {
                case "toString" -> declarationType.getName() + " declaration";
                case "hashCode" -> System.identityHashCode(proxy);
                case "equals" -> proxy == (args != null && args.length == 1 ? args[0] : null);
                default -> throw new UnsupportedOperationException(method.toString());
            };
        }

        private Object invokeDefault(Object proxy, Method method, Object[] args) throws Throwable {
            Object[] safeArgs = args != null ? args : new Object[0];
            Class<?> declaringClass = method.getDeclaringClass();
            MethodHandles.Lookup lookup = MethodHandles.privateLookupIn(declaringClass, MethodHandles.lookup());
            return lookup.findSpecial(
                    declaringClass,
                    method.getName(),
                    MethodType.methodType(method.getReturnType(), method.getParameterTypes()),
                    declaringClass)
                .bindTo(proxy)
                .invokeWithArguments(safeArgs);
        }
    }
}
