package ca.frc6390.athena.mechanism.core;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;

/**
 * Discovers hooks from arbitrary already-constructed robot objects.
 */
public final class HookIntrospector {
    private static final Map<Class<?>, Field[]> FIELDS_BY_TYPE = new ConcurrentHashMap<>();

    private HookIntrospector() {
    }

    /**
     * Inspects an object graph using the root class name as the root path.
     *
     * @param root root object
     * @return hook bindings keyed by field path
     */
    public static Map<String, HookBinding> inspect(Object root) {
        Objects.requireNonNull(root, "root");
        return inspect(defaultName(root.getClass()), root);
    }

    /**
     * Inspects an object graph.
     *
     * @param rootName root path name
     * @param root root object
     * @return hook bindings keyed by field path
     */
    public static Map<String, HookBinding> inspect(String rootName, Object root) {
        Objects.requireNonNull(root, "root");
        Map<String, HookBinding> hooks = new LinkedHashMap<>();
        Set<Object> visited = Collections.newSetFromMap(new IdentityHashMap<>());
        inspectInto(rootName == null || rootName.isBlank() ? "root" : rootName, root, hooks, visited);
        return Collections.unmodifiableMap(hooks);
    }

    private static void inspectInto(
            String path,
            Object instance,
            Map<String, HookBinding> hooks,
            Set<Object> visited) {
        if (instance == null || visited.contains(instance) || !shouldDescend(instance)) {
            return;
        }
        visited.add(instance);
        for (Field field : fields(instance.getClass())) {
            if (field.isSynthetic() || Modifier.isStatic(field.getModifiers())) {
                continue;
            }
            Object value = read(field, instance);
            if (value == null) {
                continue;
            }
            String fieldPath = path + "." + field.getName();
            if (value instanceof HookBinding hook) {
                hooks.put(fieldPath, hook);
            } else {
                inspectInto(fieldPath, value, hooks, visited);
            }
        }
    }

    private static Field[] fields(Class<?> type) {
        return FIELDS_BY_TYPE.computeIfAbsent(type, HookIntrospector::discoverFields);
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
            if (!field.canAccess(instance)) {
                field.setAccessible(true);
            }
            return field.get(instance);
        } catch (IllegalAccessException exception) {
            throw new IllegalStateException("Unable to inspect field " + field.getName(), exception);
        }
    }

    private static boolean shouldDescend(Object value) {
        Class<?> type = value.getClass();
        if (type.isPrimitive()
                || type.isEnum()
                || type.isArray()
                || value instanceof String
                || value instanceof Number
                || value instanceof Boolean
                || value instanceof Character
                || value instanceof Runnable
                || value instanceof HookBinding
                || value instanceof EventBinding
                || value instanceof Action) {
            return false;
        }
        Package typePackage = type.getPackage();
        if (typePackage == null) {
            return true;
        }
        String packageName = typePackage.getName();
        return !packageName.startsWith("java.")
                && !packageName.startsWith("javax.")
                && !packageName.startsWith("jdk.")
                && !packageName.startsWith("edu.wpi.")
                && !packageName.startsWith("com.ctre.")
                && !packageName.startsWith("com.revrobotics.")
                && !packageName.startsWith("ca.frc6390.athena.hardware.")
                && !packageName.startsWith("ca.frc6390.athena.mechanism.ref.");
    }

    private static String defaultName(Class<?> type) {
        String simpleName = type.getSimpleName();
        if (simpleName.isBlank()) {
            return "root";
        }
        return simpleName.substring(0, 1).toLowerCase(Locale.ROOT) + simpleName.substring(1);
    }
}
