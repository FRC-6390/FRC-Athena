package ca.frc6390.athena.mechanism.core;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Objects;
import java.util.Set;
import java.util.concurrent.ConcurrentHashMap;
import java.util.Map;

/**
 * Discovers path states from autonomous declaration objects.
 */
public final class PathIntrospector {
    private static final Map<Class<?>, List<Field>> FIELDS_BY_TYPE = new ConcurrentHashMap<>();

    private PathIntrospector() {
    }

    /**
     * Finds every {@link PathState} reachable from a declaration root.
     *
     * @param root autonomous declaration root
     * @return discovered path states in declaration order
     */
    public static List<PathState> inspect(Object root) {
        Objects.requireNonNull(root, "root");
        List<PathState> paths = new ArrayList<>();
        Set<Object> visited = Collections.newSetFromMap(new IdentityHashMap<>());
        collect(root, paths, visited);
        return List.copyOf(paths);
    }

    private static void collect(Object value, List<PathState> paths, Set<Object> visited) {
        if (value == null || !visited.add(value)) {
            return;
        }
        if (value instanceof PathState path) {
            paths.add(path);
            return;
        }
        Class<?> type = value.getClass();
        if (shouldNotRecurse(type) || value instanceof Mechanism || value instanceof State) {
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
        return FIELDS_BY_TYPE.computeIfAbsent(type, PathIntrospector::discoverFields);
    }

    private static List<Field> discoverFields(Class<?> type) {
        List<Field> fields = new ArrayList<>();
        Class<?> cursor = type;
        while (cursor != null && cursor != Object.class) {
            fields.addAll(List.of(cursor.getDeclaredFields()));
            cursor = cursor.getSuperclass();
        }
        return List.copyOf(fields);
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
