package ca.frc6390.athena.robot.explore;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Predicate;

/**
 * Delta debugging (ddmin) over an ordered list of input events. Finds a 1-minimal subsequence for which the
 * predicate still holds: removing any single remaining element makes it fail.
 */
final class Shrinker {
    private Shrinker() {
    }

    /**
     * Minimizes {@code items}.
     *
     * @param items sequence for which {@code reproduces} holds
     * @param reproduces returns true when a candidate subsequence still shows the finding
     * @param <E> element type
     * @return a minimal subsequence in the original order
     */
    static <E> List<E> minimize(List<E> items, Predicate<List<E>> reproduces) {
        if (reproduces.test(List.of())) {
            return List.of();
        }
        List<E> current = new ArrayList<>(items);
        int granularity = 2;
        while (current.size() >= 2) {
            int chunk = (current.size() + granularity - 1) / granularity;
            boolean reduced = false;
            for (int start = 0; start < current.size(); start += chunk) {
                int end = Math.min(current.size(), start + chunk);
                List<E> complement = new ArrayList<>(current.size() - (end - start));
                complement.addAll(current.subList(0, start));
                complement.addAll(current.subList(end, current.size()));
                if (reproduces.test(complement)) {
                    current = complement;
                    granularity = Math.max(granularity - 1, 2);
                    reduced = true;
                    break;
                }
            }
            if (!reduced) {
                if (granularity >= current.size()) {
                    break;
                }
                granularity = Math.min(granularity * 2, current.size());
            }
        }
        return List.copyOf(current);
    }
}
