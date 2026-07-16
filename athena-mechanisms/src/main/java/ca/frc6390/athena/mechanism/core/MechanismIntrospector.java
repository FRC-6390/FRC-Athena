package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.ImuDevice;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.mechanism.control.FeedforwardGains;
import ca.frc6390.athena.mechanism.control.PidGains;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.util.LinkedHashMap;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Discovers mechanism structure from already-constructed mechanism instances.
 */
final class MechanismIntrospector {
    private static final Map<Class<?>, Field[]> FIELDS_BY_TYPE = new ConcurrentHashMap<>();
    private static final Map<Class<?>, Method[]> METHODS_BY_TYPE = new ConcurrentHashMap<>();

    private MechanismIntrospector() {
    }

    static MechanismNode inspect(Mechanism mechanism) {
        Objects.requireNonNull(mechanism, "mechanism");
        return inspect(defaultName(mechanism.getClass()), mechanism);
    }

    static MechanismNode inspect(String name, Mechanism mechanism) {
        Objects.requireNonNull(mechanism, "mechanism");
        Map<String, Mechanism> children = new LinkedHashMap<>();
        Map<String, Action> actions = new LinkedHashMap<>();
        Map<String, Object> declarations = new LinkedHashMap<>();
        Map<String, HookBinding> hooks = new LinkedHashMap<>();

        for (Field field : fields(mechanism.getClass())) {
            if (field.isSynthetic()) {
                continue;
            }
            String fieldName = field.getName();
            Telemetry telemetry = field.getAnnotation(Telemetry.class);
            if (telemetry != null) {
                putDeclaration(
                        declarations,
                        AnnotatedTelemetry.name(fieldName, telemetry),
                        AnnotatedTelemetry.field(field, mechanism, telemetry));
                continue;
            }
            Object value = read(field, mechanism);
            if (value == null) {
                continue;
            }
            if (value instanceof Mechanism child && !Modifier.isStatic(field.getModifiers())) {
                children.put(fieldName, child);
            } else if (value instanceof Action action) {
                actions.put(fieldName, action);
            } else if (value instanceof HookBinding hook) {
                hooks.put(fieldName, hook);
            } else if (value instanceof HookGroup group) {
                group.hooks().forEach((hookName, hook) -> hooks.put(fieldName + "." + hookName, hook));
            } else {
                Object declaration = declaration(value);
                if (declaration != null) {
                    putDeclaration(declarations, fieldName, declaration);
                }
            }
        }

        for (Method method : methods(mechanism.getClass())) {
            Telemetry telemetry = method.getAnnotation(Telemetry.class);
            if (telemetry == null || method.isSynthetic() || method.isBridge()) continue;
            String telemetryName = AnnotatedTelemetry.name(method.getName(), telemetry);
            putDeclaration(declarations, telemetryName, AnnotatedTelemetry.method(method, mechanism, telemetry));
        }

        return new MechanismNode(name, mechanism, children, actions, declarations, hooks);
    }

    private static Field[] fields(Class<?> type) {
        return FIELDS_BY_TYPE.computeIfAbsent(type, MechanismIntrospector::discoverFields);
    }

    private static Method[] methods(Class<?> type) {
        return METHODS_BY_TYPE.computeIfAbsent(type, MechanismIntrospector::discoverMethods);
    }

    private static Field[] discoverFields(Class<?> type) {
        Map<String, Field> fields = new LinkedHashMap<>();
        Class<?> current = type;
        while (current != null && current != Object.class) {
            for (Field field : current.getDeclaredFields()) {
                fields.putIfAbsent(field.getName(), field);
            }
            current = current.getSuperclass();
        }
        return fields.values().toArray(Field[]::new);
    }

    private static Method[] discoverMethods(Class<?> type) {
        Map<String, Method> methods = new LinkedHashMap<>();
        Class<?> current = type;
        while (current != null && current != Object.class) {
            for (Method method : current.getDeclaredMethods()) {
                String signature = method.getName() + java.util.Arrays.toString(method.getParameterTypes());
                methods.putIfAbsent(signature, method);
            }
            current = current.getSuperclass();
        }
        return methods.values().toArray(Method[]::new);
    }

    private static void putDeclaration(Map<String, Object> declarations, String name, Object declaration) {
        if (declarations.putIfAbsent(name, declaration) != null) {
            throw new IllegalArgumentException("Duplicate mechanism declaration path '" + name + "'.");
        }
    }

    private static Object read(Field field, Object instance) {
        try {
            if (!field.canAccess(Modifier.isStatic(field.getModifiers()) ? null : instance)) {
                field.setAccessible(true);
            }
            return field.get(Modifier.isStatic(field.getModifiers()) ? null : instance);
        } catch (IllegalAccessException exception) {
            throw new IllegalStateException("Unable to inspect field " + field.getName(), exception);
        }
    }

    private static Object declaration(Object value) {
        if (value instanceof Slot<?, ?> slot) {
            return slot.filled() ? declaration(slot.get()) : null;
        }
        if (value instanceof Iterable<?> values) {
            List<Object> declarations = new ArrayList<>();
            for (Object element : values) {
                Object declaration = declaration(element);
                if (declaration != null) {
                    declarations.add(declaration);
                }
            }
            return declarations.isEmpty() ? null : List.copyOf(declarations);
        }
        return isDeclaration(value) ? value : null;
    }

    private static boolean isDeclaration(Object value) {
        return value instanceof MotorDevice
                || value instanceof EncoderDevice
                || value instanceof ImuDevice
                || value instanceof DigitalInputDevice
                || value instanceof Range
                || value instanceof GearRatio
                || value instanceof PidGains
                || value instanceof FeedforwardGains
                || value instanceof ControlBinding
                || value instanceof PathAction
                || value instanceof SimModel
                || value instanceof SimModel.Source
                || value instanceof TelemetryValue
                || value instanceof TelemetrySource;
    }

    private static String defaultName(Class<?> type) {
        String simpleName = type.getSimpleName();
        if (simpleName.isBlank()) {
            return "mechanism";
        }
        return simpleName.substring(0, 1).toLowerCase(Locale.ROOT) + simpleName.substring(1);
    }
}
