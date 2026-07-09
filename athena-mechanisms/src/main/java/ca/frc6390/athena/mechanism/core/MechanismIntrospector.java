package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.device.DigitalInputDevice;
import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.device.MotorDevice;
import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.hardware.sim.SimModel;
import ca.frc6390.athena.mechanism.ref.FeedforwardGains;
import ca.frc6390.athena.mechanism.ref.PidGains;
import java.lang.reflect.Field;
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
public final class MechanismIntrospector {
    private static final Map<Class<?>, Field[]> FIELDS_BY_TYPE = new ConcurrentHashMap<>();

    private MechanismIntrospector() {
    }

    public static MechanismNode inspect(Mechanism mechanism) {
        Objects.requireNonNull(mechanism, "mechanism");
        return inspect(defaultName(mechanism.getClass()), mechanism);
    }

    public static MechanismNode inspect(String name, Mechanism mechanism) {
        Objects.requireNonNull(mechanism, "mechanism");
        Map<String, Mechanism> children = new LinkedHashMap<>();
        Map<String, Action> actions = new LinkedHashMap<>();
        Map<String, Object> declarations = new LinkedHashMap<>();
        Map<String, HookBinding> hooks = new LinkedHashMap<>();

        for (Field field : fields(mechanism.getClass())) {
            if (field.isSynthetic()) {
                continue;
            }
            Object value = read(field, mechanism);
            if (value == null) {
                continue;
            }
            String fieldName = field.getName();
            if (value instanceof Mechanism child && !Modifier.isStatic(field.getModifiers())) {
                children.put(fieldName, child);
            } else if (value instanceof Action action) {
                actions.put(fieldName, action);
            } else if (value instanceof HookBinding hook) {
                hooks.put(fieldName, hook);
            } else {
                Object declaration = declaration(value);
                if (declaration != null) {
                    declarations.put(fieldName, declaration);
                }
            }
        }

        return new MechanismNode(name, mechanism, children, actions, declarations, hooks);
    }

    private static Field[] fields(Class<?> type) {
        return FIELDS_BY_TYPE.computeIfAbsent(type, MechanismIntrospector::discoverFields);
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
                || value instanceof DigitalInputDevice
                || value instanceof Range
                || value instanceof GearRatio
                || value instanceof PidGains
                || value instanceof FeedforwardGains
                || value instanceof ControlBinding
                || value instanceof PathAction
                || value instanceof SimModel;
    }

    private static String defaultName(Class<?> type) {
        String simpleName = type.getSimpleName();
        if (simpleName.isBlank()) {
            return "mechanism";
        }
        return simpleName.substring(0, 1).toLowerCase(Locale.ROOT) + simpleName.substring(1);
    }
}
