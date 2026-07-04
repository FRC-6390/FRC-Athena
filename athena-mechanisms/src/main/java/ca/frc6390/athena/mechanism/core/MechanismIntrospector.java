package ca.frc6390.athena.mechanism.core;

import ca.frc6390.athena.hardware.ref.AnalogInputRef;
import ca.frc6390.athena.hardware.ref.AxisRef;
import ca.frc6390.athena.hardware.ref.BooleanRef;
import ca.frc6390.athena.hardware.ref.ButtonRef;
import ca.frc6390.athena.hardware.ref.ControllerRef;
import ca.frc6390.athena.hardware.ref.DigitalInputRef;
import ca.frc6390.athena.hardware.ref.EncoderRef;
import ca.frc6390.athena.hardware.ref.GearRatioRef;
import ca.frc6390.athena.hardware.ref.MotorRef;
import ca.frc6390.athena.hardware.ref.NumberRef;
import ca.frc6390.athena.hardware.ref.RangeRef;
import ca.frc6390.athena.hardware.ref.SimRef;
import ca.frc6390.athena.mechanism.ref.FeedforwardRef;
import ca.frc6390.athena.mechanism.ref.PidRef;
import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;

/**
 * Discovers mechanism structure from already-constructed mechanism instances.
 */
public final class MechanismIntrospector {
    private MechanismIntrospector() {
    }

    /**
     * Inspects a mechanism using its class name as the root name.
     *
     * @param mechanism mechanism instance
     * @return mechanism definition
     */
    public static MechanismDefinition inspect(Mechanism mechanism) {
        Objects.requireNonNull(mechanism, "mechanism");
        return inspect(defaultName(mechanism.getClass()), mechanism);
    }

    /**
     * Inspects a mechanism.
     *
     * @param name root name
     * @param mechanism mechanism instance
     * @return mechanism definition
     */
    public static MechanismDefinition inspect(String name, Mechanism mechanism) {
        Objects.requireNonNull(mechanism, "mechanism");
        Map<String, Mechanism> children = new LinkedHashMap<>();
        Map<String, MechanismState> states = new LinkedHashMap<>();
        Map<String, Object> refs = new LinkedHashMap<>();
        String initialStateName = "";
        MechanismState initialState = null;

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
            } else if (value instanceof MechanismState state) {
                states.put(fieldName, state);
                if (field.isAnnotationPresent(InitialState.class)) {
                    if (initialState != null) {
                        throw new IllegalStateException(
                                "Mechanism " + name + " declares more than one @InitialState field.");
                    }
                    initialStateName = fieldName;
                    initialState = state;
                }
            } else if (isRef(value)) {
                refs.put(fieldName, value);
            }
        }

        return new MechanismDefinition(name, mechanism, children, states, initialStateName, initialState, refs);
    }

    private static Field[] fields(Class<?> type) {
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

    private static boolean isRef(Object value) {
        return value instanceof MotorRef
                || value instanceof EncoderRef
                || value instanceof DigitalInputRef
                || value instanceof AnalogInputRef
                || value instanceof NumberRef
                || value instanceof BooleanRef
                || value instanceof ControllerRef
                || value instanceof AxisRef
                || value instanceof ButtonRef
                || value instanceof RangeRef
                || value instanceof GearRatioRef
                || value instanceof PidRef
                || value instanceof FeedforwardRef
                || value instanceof ControlRef
                || value instanceof SimRef;
    }

    private static String defaultName(Class<?> type) {
        String simpleName = type.getSimpleName();
        if (simpleName.isBlank()) {
            return "mechanism";
        }
        return simpleName.substring(0, 1).toLowerCase(Locale.ROOT) + simpleName.substring(1);
    }
}
