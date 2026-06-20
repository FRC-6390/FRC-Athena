package ca.frc6390.athena.superstructure.runtime;

import java.util.OptionalDouble;

/**
 * Leaf mechanism target produced by a superstructure transition plan.
 *
 * @param path stable dotted path from the planned superstructure to the part
 * @param mechanismName mechanism spec name
 * @param stateName requested mechanism state name
 * @param target numeric mechanism target, or {@link Double#NaN} when absent
 */
public record SuperstructureMechanismTarget(
        String path,
        String mechanismName,
        String stateName,
        double target) {
    public SuperstructureMechanismTarget {
        path = path == null || path.isBlank() ? "part" : path;
        mechanismName = mechanismName == null || mechanismName.isBlank() ? "mechanism" : mechanismName;
        stateName = stateName == null || stateName.isBlank() ? "state" : stateName;
    }

    /**
     * Returns the numeric target when the mechanism state declares one.
     *
     * @return optional numeric target
     */
    public OptionalDouble targetValue() {
        return Double.isNaN(target) ? OptionalDouble.empty() : OptionalDouble.of(target);
    }
}
