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
 * Discovers path Actions from autonomous declaration objects.
 */
final class PathIntrospector {
    private static final Map<Class<?>, List<Field>> FIELDS_BY_TYPE = new ConcurrentHashMap<>();

    private PathIntrospector() {
    }

    /**
     * Finds every {@link PathAction} reachable from a declaration root.
     *
     * @param root autonomous declaration root
     * @return discovered path Actions in declaration order
     */
    public static List<PathAction> inspect(Object root) {
        Objects.requireNonNull(root, "root");
        List<PathAction> paths = new ArrayList<>();
        Set<Object> visited = Collections.newSetFromMap(new IdentityHashMap<>());
        collect(root, paths, visited);
        return List.copyOf(paths);
    }

    private static void collect(Object value, List<PathAction> paths, Set<Object> visited) {
        if (value == null || !visited.add(value)) {
            return;
        }
        if (value instanceof PathAction path) {
            paths.add(path);
            path.markers().values().forEach(marker -> collect(marker, paths, visited));
            return;
        }
        if (value instanceof Actions.Sequence sequence) {
            sequence.steps().forEach(step -> collect(step.action(), paths, visited));
            collect(sequence.next(), paths, visited);
            return;
        }
        if (value instanceof Actions.Cycle cycle) {
            cycle.steps().forEach(step -> collect(step.action(), paths, visited));
            return;
        }
        if (value instanceof Actions.Parallel parallel) {
            parallel.Actions().forEach(action -> collect(action, paths, visited));
            return;
        }
        if (value instanceof Actions.Race race) {
            race.Actions().forEach(action -> collect(action, paths, visited));
            return;
        }
        if (value instanceof Actions.Deadline deadline) {
            collect(deadline.primary(), paths, visited);
            deadline.others().forEach(action -> collect(action, paths, visited));
            return;
        }
        if (value instanceof Actions.Choice choice) {
            collect(choice.active(), paths, visited);
            collect(choice.inactive(), paths, visited);
            return;
        }
        if (value instanceof Actions.WhenBranch branch) {
            collect(branch.active(), paths, visited);
            return;
        }
        if (value instanceof Actions.Timeout timeout) {
            collect(timeout.action(), paths, visited);
            return;
        }
        if (value instanceof Actions.Then then) {
            collect(then.action(), paths, visited);
            collect(then.next(), paths, visited);
            return;
        }
        if (value instanceof Action.Then then) {
            collect(then.action(), paths, visited);
            collect(then.next(), paths, visited);
            return;
        }
        if (value instanceof Actions.Conditional conditional) {
            collect(conditional.action(), paths, visited);
            collect(conditional.next(), paths, visited);
            return;
        }
        if (value instanceof Action.Conditional conditional) {
            collect(conditional.action(), paths, visited);
            collect(conditional.next(), paths, visited);
            return;
        }
        if (value instanceof Actions.WithinTolerance within) {
            collect(within.action(), paths, visited);
            return;
        }
        Class<?> type = value.getClass();
        if (shouldNotRecurse(type) || value instanceof Mechanism || value instanceof Action) {
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
