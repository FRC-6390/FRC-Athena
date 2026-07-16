package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.runtime.geometry.Geometry2d;
import java.lang.reflect.Field;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;

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
                    () -> ((Number) read(field, instance)).doubleValue(),
                    value -> writeNumber(field, instance, clamp(value, annotation.min(), annotation.max())));
        }
        if (type == boolean.class || type == Boolean.class) {
            if (hasBounds(annotation)) throw invalid(field, "boolean telemetry cannot have numeric bounds");
            return TelemetryValue.writableBoolean(
                    () -> (Boolean) read(field, instance),
                    value -> write(field, instance, value));
        }
        throw invalid(field, "writable telemetry supports only numeric and boolean fields");
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
        return readOnly(method.getReturnType(), () -> invoke(method, instance), method);
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

    private static Object invoke(Method method, Object instance) {
        try {
            return method.invoke(Modifier.isStatic(method.getModifiers()) ? null : instance);
        } catch (IllegalAccessException exception) {
            throw new IllegalStateException("Unable to invoke telemetry method " + method.getName(), exception);
        } catch (InvocationTargetException exception) {
            Throwable cause = exception.getCause();
            if (cause instanceof RuntimeException runtime) throw runtime;
            if (cause instanceof Error error) throw error;
            throw new IllegalStateException("Telemetry method " + method.getName() + " failed", cause);
        }
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
