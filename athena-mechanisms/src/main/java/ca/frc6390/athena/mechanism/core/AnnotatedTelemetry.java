package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.runtime.geometry.Geometry2d;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.lang.invoke.MethodHandle;
import java.lang.invoke.MethodHandles;
import java.lang.invoke.MethodType;

/** Converts cached annotated members into the existing telemetry runtime representation. */
final class AnnotatedTelemetry {
    private AnnotatedTelemetry() {
    }

    static TelemetryValue field(Field field, Object instance, Telemetry annotation) {
        makeAccessible(field, instance);
        Class<?> type = field.getType();
        if (TelemetryValue.class.isAssignableFrom(type)) {
            if (annotation.writable() || hasBounds(annotation)) {
                throw invalid(field, "TelemetryValue controls its own writability and bounds");
            }
            Object value = read(field, instance);
            if (value == null) throw invalid(field, "annotated TelemetryValue cannot be null");
            return (TelemetryValue) value;
        }

        if (!annotation.writable()) {
            if (type.isPrimitive() && isNumber(type)) {
                return TelemetryValue.number(() -> readPrimitiveNumber(field, instance));
            }
            if (type == boolean.class) {
                return TelemetryValue.bool(() -> readPrimitiveBoolean(field, instance));
            }
            return readOnly(type, () -> read(field, instance), field);
        }
        if (Modifier.isFinal(field.getModifiers())) {
            throw invalid(field, "writable telemetry field cannot be final");
        }
        if (Modifier.isStatic(field.getModifiers())) {
            throw invalid(field, "writable telemetry field cannot be static");
        }
        if (isNumber(type)) {
            requireBounds(annotation, field);
            return TelemetryValue.writableNumber(
                    type.isPrimitive()
                            ? () -> readPrimitiveNumber(field, instance)
                            : () -> ((Number) read(field, instance)).doubleValue(),
                    value -> writeNumber(field, instance, clamp(value, annotation.min(), annotation.max())));
        }
        if (type == boolean.class || type == Boolean.class) {
            if (hasBounds(annotation)) throw invalid(field, "boolean telemetry cannot have numeric bounds");
            return TelemetryValue.writableBoolean(
                    type == boolean.class
                            ? () -> readPrimitiveBoolean(field, instance)
                            : () -> (Boolean) read(field, instance),
                    value -> write(field, instance, value));
        }
        if (type == String.class) {
            if (hasBounds(annotation)) throw invalid(field, "string telemetry cannot have numeric bounds");
            return TelemetryValue.writableString(
                    () -> String.valueOf(read(field, instance)),
                    value -> write(field, instance, value));
        }
        if (type.isEnum()) {
            if (hasBounds(annotation)) throw invalid(field, "enum telemetry cannot have numeric bounds");
            return TelemetryValue.writableString(
                    () -> String.valueOf(read(field, instance)),
                    value -> write(field, instance, enumValue(type, value, field)));
        }
        throw invalid(field, "writable telemetry supports numeric, boolean, string, and enum fields");
    }

    static TelemetryValue method(Method method, Object instance, Telemetry annotation) {
        if (method.getParameterCount() != 0) {
            throw invalid(method, "telemetry method must have no parameters");
        }
        if (method.getReturnType() == void.class) {
            throw invalid(method, "telemetry method must return a value");
        }
        if (annotation.writable()) {
            throw invalid(method, "telemetry method cannot be writable");
        }
        if (hasBounds(annotation)) {
            throw invalid(method, "read-only telemetry method cannot have writable bounds");
        }
        makeAccessible(method, instance);
        MethodHandle handle = methodHandle(method, instance);
        Class<?> type = method.getReturnType();
        if (isNumber(type)) {
            MethodHandle numeric = handle.asType(MethodType.methodType(double.class));
            return TelemetryValue.number(() -> invokeNumber(numeric, method));
        }
        if (type == boolean.class || type == Boolean.class) {
            MethodHandle bool = handle.asType(MethodType.methodType(boolean.class));
            return TelemetryValue.bool(() -> invokeBoolean(bool, method));
        }
        MethodHandle object = handle.asType(MethodType.methodType(Object.class));
        return readOnly(type, () -> invokeObject(object, method), method);
    }

    static String name(String fallback, Telemetry annotation) {
        String requested = annotation.value().isBlank() ? fallback : annotation.value().trim();
        if (requested.startsWith("/") || requested.endsWith("/")
                || java.util.Arrays.stream(requested.split("/", -1)).anyMatch(String::isBlank)) {
            throw new IllegalArgumentException("Invalid telemetry path '" + requested + "'.");
        }
        return requested;
    }

    private static TelemetryValue readOnly(
            Class<?> type, java.util.function.Supplier<Object> reader, java.lang.reflect.Member member) {
        if (TelemetryValue.class.isAssignableFrom(type)) {
            Object value = reader.get();
            if (value == null) throw invalid(member, "annotated TelemetryValue cannot be null");
            return (TelemetryValue) value;
        }
        if (isNumber(type)) {
            return TelemetryValue.number(() -> ((Number) reader.get()).doubleValue());
        }
        if (type == boolean.class || type == Boolean.class) {
            return TelemetryValue.bool(() -> (Boolean) reader.get());
        }
        if (type == String.class || CharSequence.class.isAssignableFrom(type) || type.isEnum()) {
            return TelemetryValue.string(() -> String.valueOf(reader.get()));
        }
        if (Geometry2d.class.isAssignableFrom(type)) {
            return TelemetryValue.dynamicGeometry(() -> (Geometry2d) reader.get());
        }
        throw invalid(member, "unsupported telemetry type " + type.getTypeName());
    }

    private static boolean isNumber(Class<?> type) {
        return type == byte.class || type == short.class || type == int.class || type == long.class
                || type == float.class || type == double.class || Number.class.isAssignableFrom(type);
    }

    @SuppressWarnings({"unchecked", "rawtypes"})
    private static Object enumValue(Class<?> type, String value, Field field) {
        String requested = value == null ? "" : value.trim();
        for (Object constant : type.getEnumConstants()) {
            if (((Enum<?>) constant).name().equalsIgnoreCase(requested)) return constant;
        }
        throw invalid(field, "unknown " + type.getSimpleName() + " value '" + requested + "'");
    }

    private static void writeNumber(Field field, Object instance, double value) {
        if (!Double.isFinite(value)) return;
        Class<?> type = field.getType();
        if (type == double.class || type == Double.class) write(field, instance, value);
        else if (type == float.class || type == Float.class) write(field, instance, (float) value);
        else if (type == long.class || type == Long.class) write(field, instance, Math.round(value));
        else if (type == int.class || type == Integer.class) write(field, instance, (int) Math.round(value));
        else if (type == short.class || type == Short.class) write(field, instance, (short) Math.round(value));
        else if (type == byte.class || type == Byte.class) write(field, instance, (byte) Math.round(value));
        else throw invalid(field, "unsupported numeric telemetry type " + type.getTypeName());
    }

    private static Object read(Field field, Object instance) {
        try {
            return field.get(Modifier.isStatic(field.getModifiers()) ? null : instance);
        } catch (IllegalAccessException exception) {
            throw new IllegalStateException("Unable to read telemetry field " + field.getName(), exception);
        }
    }

    private static void write(Field field, Object instance, Object value) {
        try {
            field.set(instance, value);
        } catch (IllegalAccessException exception) {
            throw new IllegalStateException("Unable to write telemetry field " + field.getName(), exception);
        }
    }

    private static MethodHandle methodHandle(Method method, Object instance) {
        try {
            MethodHandle handle = MethodHandles.lookup().unreflect(method);
            return Modifier.isStatic(method.getModifiers()) ? handle : handle.bindTo(instance);
        } catch (IllegalAccessException exception) {
            throw new IllegalStateException("Unable to bind telemetry method " + method.getName(), exception);
        }
    }

    private static double readPrimitiveNumber(Field field, Object instance) {
        try {
            return field.getDouble(Modifier.isStatic(field.getModifiers()) ? null : instance);
        } catch (IllegalAccessException exception) {
            throw new IllegalStateException("Unable to read telemetry field " + field.getName(), exception);
        }
    }

    private static boolean readPrimitiveBoolean(Field field, Object instance) {
        try {
            return field.getBoolean(Modifier.isStatic(field.getModifiers()) ? null : instance);
        } catch (IllegalAccessException exception) {
            throw new IllegalStateException("Unable to read telemetry field " + field.getName(), exception);
        }
    }

    private static double invokeNumber(MethodHandle handle, Method method) {
        try { return (double) handle.invokeExact(); }
        catch (Throwable failure) { throw invocationFailure(method, failure); }
    }

    private static boolean invokeBoolean(MethodHandle handle, Method method) {
        try { return (boolean) handle.invokeExact(); }
        catch (Throwable failure) { throw invocationFailure(method, failure); }
    }

    private static Object invokeObject(MethodHandle handle, Method method) {
        try { return (Object) handle.invokeExact(); }
        catch (Throwable failure) { throw invocationFailure(method, failure); }
    }

    private static RuntimeException invocationFailure(Method method, Throwable failure) {
        if (failure instanceof RuntimeException runtime) return runtime;
        if (failure instanceof Error error) throw error;
        return new IllegalStateException("Telemetry method " + method.getName() + " failed", failure);
    }

    private static void makeAccessible(Field field, Object instance) {
        Object target = Modifier.isStatic(field.getModifiers()) ? null : instance;
        if (!field.canAccess(target)) field.setAccessible(true);
    }

    private static void makeAccessible(Method method, Object instance) {
        Object target = Modifier.isStatic(method.getModifiers()) ? null : instance;
        if (!method.canAccess(target)) method.setAccessible(true);
    }

    private static boolean hasBounds(Telemetry annotation) {
        return annotation.min() != -Double.MAX_VALUE || annotation.max() != Double.MAX_VALUE;
    }

    private static void requireBounds(Telemetry annotation, java.lang.reflect.Member member) {
        if (!Double.isFinite(annotation.min()) || !Double.isFinite(annotation.max())
                || annotation.min() > annotation.max()) {
            throw invalid(member, "telemetry bounds must be finite and min must not exceed max");
        }
    }

    private static double clamp(double value, double minimum, double maximum) {
        return Math.max(minimum, Math.min(maximum, value));
    }

    private static IllegalArgumentException invalid(java.lang.reflect.Member member, String message) {
        return new IllegalArgumentException(
                "Invalid @Telemetry on " + member.getDeclaringClass().getName() + "." + member.getName()
                        + ": " + message + ".");
    }
}
