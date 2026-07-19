package ca.frc6390.athena.auto;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.PathAction;
import java.util.ArrayList;
import java.util.Collections;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Set;

/** Static, side-effect-free description of an autonomous Action tree. */
public record AutoPlan(List<String> steps, List<PathAction> paths) {
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
            steps.add(prefix + "↳ " + label(action) + " (shared)");
            return;
        }
        if (action instanceof PathAction path) {
            steps.add(prefix + "PATH " + path.key()
                    + (path.onlyResetsPose() ? " [reset pose only]"
                            : path.resetsOdometry() ? " [reset pose]" : ""));
            paths.add(path);
            path.markers().forEach((name, marker) -> {
                steps.add(prefix + "  MARKER " + name);
                collect(marker, prefix + "    ", steps, paths, visited);
            });
        } else if (action instanceof Actions.Sequence sequence) {
            steps.add(prefix + "SEQUENCE");
            sequence.steps().forEach(step -> collect(step.action(), prefix + "  ", steps, paths, visited));
            collect(sequence.next(), prefix + "  THEN ", steps, paths, visited);
        } else if (action instanceof Actions.Cycle cycle) {
            steps.add(prefix + "CYCLE");
            cycle.steps().forEach(step -> collect(step.action(), prefix + "  ", steps, paths, visited));
        } else if (action instanceof Actions.Parallel parallel) {
            collectGroup("PARALLEL", parallel.Actions(), prefix, steps, paths, visited);
        } else if (action instanceof Actions.Race race) {
            collectGroup("RACE", race.Actions(), prefix, steps, paths, visited);
        } else if (action instanceof Actions.Deadline deadline) {
            steps.add(prefix + "DEADLINE");
            collect(deadline.primary(), prefix + "  PRIMARY ", steps, paths, visited);
            deadline.others().forEach(other -> collect(other, prefix + "  WITH ", steps, paths, visited));
        } else if (action instanceof Actions.Choice choice) {
            steps.add(prefix + "CONDITIONAL");
            collect(choice.active(), prefix + "  TRUE ", steps, paths, visited);
            collect(choice.inactive(), prefix + "  FALSE ", steps, paths, visited);
        } else if (action instanceof Actions.WhenBranch branch) {
            steps.add(prefix + "CONDITIONAL");
            collect(branch.active(), prefix + "  TRUE ", steps, paths, visited);
        } else if (action instanceof Actions.Timeout timeout) {
            steps.add(prefix + "TIMEOUT " + timeout.seconds() + "s");
            collect(timeout.action(), prefix + "  ", steps, paths, visited);
        } else if (action instanceof Actions.Then then) {
            collect(then.action(), prefix, steps, paths, visited);
            collect(then.next(), prefix + "THEN ", steps, paths, visited);
        } else if (action instanceof Action.Then then) {
            collect(then.action(), prefix, steps, paths, visited);
            collect(then.next(), prefix + "THEN ", steps, paths, visited);
        } else if (action instanceof Actions.Conditional conditional) {
            steps.add(prefix + "UNTIL " + label(conditional.action()));
            collect(conditional.action(), prefix + "  ", steps, paths, visited);
            collect(conditional.next(), prefix + "THEN ", steps, paths, visited);
        } else if (action instanceof Action.Conditional conditional) {
            steps.add(prefix + "UNTIL " + label(conditional.action()));
            collect(conditional.action(), prefix + "  ", steps, paths, visited);
            collect(conditional.next(), prefix + "THEN ", steps, paths, visited);
        } else if (action instanceof Actions.WithinTolerance within) {
            steps.add(prefix + "WITHIN " + within.tolerance());
            collect(within.action(), prefix + "  ", steps, paths, visited);
        } else {
            steps.add(prefix + label(action));
        }
    }

    private static void collectGroup(String name, List<Action> actions, String prefix,
            List<String> steps, List<PathAction> paths, Set<Action> visited) {
        steps.add(prefix + name);
        actions.forEach(child -> collect(child, prefix + "  ", steps, paths, visited));
    }

    private static String label(Action action) {
        if (action == null) return "complete";
        String name = action.getClass().getSimpleName();
        return name == null || name.isBlank() ? action.getClass().getName() : name;
    }
}
