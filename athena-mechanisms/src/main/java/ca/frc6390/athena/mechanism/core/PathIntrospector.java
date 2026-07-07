package ca.frc6390.athena.mechanism.core;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Objects;
import java.util.Set;

/**
 * Discovers path refs from autonomous declaration objects.
 */
public final class PathIntrospector {
    private PathIntrospector() {
    }

    /**
     * Finds every {@link PathRef} reachable from a declaration root.
     *
     * @param root autonomous declaration root
     * @return discovered path refs in declaration order
     */
    public static List<PathRef> inspect(Object root) {
        Objects.requireNonNull(root, "root");
        List<PathRef> paths = new ArrayList<>();
        Set<Object> visited = Collections.newSetFromMap(new IdentityHashMap<>());
        collect(root, paths, visited);
        return List.copyOf(paths);
    }

    private static void collect(Object value, List<PathRef> paths, Set<Object> visited) {
        if (value == null || !visited.add(value)) {
            return;
        }
        if (value instanceof PathRef path) {
            paths.add(path);
            return;
        }
        Class<?> type = value.getClass();
        if (shouldNotRecurse(type) || value instanceof Mechanism || value instanceof MechanismState) {
            return;
        }
        for (Field field : fields(type)) {
            if (Modifier.isStatic(field.getModifiers())) {
                continue;
            }
            try {
                if (!field.canAccess(value)) {
                    field.setAccessible(true);
                }
                collect(field.get(value), paths, visited);
            } catch (IllegalAccessException exception) {
                throw new IllegalStateException("Unable to inspect path field " + field.getName(), exception);
            }
        }
    }

    private static List<Field> fields(Class<?> type) {
        List<Field> fields = new ArrayList<>();
        Class<?> cursor = type;
        while (cursor != null && cursor != Object.class) {
            fields.addAll(List.of(cursor.getDeclaredFields()));
            cursor = cursor.getSuperclass();
        }
        return fields;
    }

    private static boolean shouldNotRecurse(Class<?> type) {
        if (type.isPrimitive() || type.isEnum() || type.isArray()) {
            return true;
        }
        Package typePackage = type.getPackage();
        String packageName = typePackage == null ? "" : typePackage.getName();
        return packageName.startsWith("java.")
                || packageName.startsWith("javax.")
                || packageName.startsWith("edu.wpi.");
    }
}
