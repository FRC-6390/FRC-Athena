package ca.frc6390.athena.auto;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.PathAction;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Set;

/**
 * Static, side-effect-free description of an autonomous Action tree. Walks the tree through
 * {@link Action#children()} and labels each node with {@link Action#describe()}, so every Action kind
 * that describes its children is listed without a type-specific branch here.
 */
public record AutoPlan(List<String> steps, List<PathAction> paths) {
    private static final String MARKER_ROLE = "marker:";

    public AutoPlan {
        steps = List.copyOf(steps);
        paths = List.copyOf(paths);
    }

    public static AutoPlan inspect(Action action) {
        List<String> steps = new ArrayList<>();
        List<PathAction> paths = new ArrayList<>();
        collect(action, "", steps, paths, Collections.newSetFromMap(new IdentityHashMap<>()));
        return new AutoPlan(steps, paths);
    }

    private static void collect(Action action, String prefix, List<String> steps,
            List<PathAction> paths, Set<Action> visited) {
        if (action == null) return;
        if (!visited.add(action)) {
            steps.add(prefix + "↳ " + action.describe() + " (shared)");
            return;
        }
        if (action instanceof PathAction path) {
            paths.add(path);
        }
        String label = action.describe();
        if (!label.isEmpty()) {
            steps.add(prefix + label);
        }
        for (Action.Child child : action.children()) {
            String role = child.role();
            if (role.startsWith(MARKER_ROLE)) {
                // Markers hang under their path, aligned to its indentation rather than its role tag.
                String indent = prefix.substring(0, prefix.length() - prefix.stripLeading().length());
                steps.add(indent + "  MARKER " + role.substring(MARKER_ROLE.length()));
                collect(child.action(), indent + "    ", steps, paths, visited);
            } else {
                collect(child.action(), prefix + indentation(role), steps, paths, visited);
            }
        }
    }

    /** Text placed before a child's label, by the role the parent gives it. */
    private static String indentation(String role) {
        return switch (role) {
            case "first" -> "";
            case "then" -> "THEN ";
            case "primary" -> "  PRIMARY ";
            case "with" -> "  WITH ";
            case "true" -> "  TRUE ";
            case "false" -> "  FALSE ";
            default -> "  ";
        };
    }
}
