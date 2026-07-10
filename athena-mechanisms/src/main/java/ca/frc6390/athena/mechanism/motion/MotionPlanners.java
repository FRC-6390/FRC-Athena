package ca.frc6390.athena.mechanism.motion;

import ca.frc6390.athena.mechanism.constraint.ConstraintContext;
import ca.frc6390.athena.mechanism.constraint.ConstraintResult;
import ca.frc6390.athena.mechanism.constraint.Constraints;

/**
 * Factories for scalar motion planners.
 */
public final class MotionPlanners {
    private MotionPlanners() {
    }

    public static MotionPlanner direct() {
        return (context, constraints) -> Constraints.evaluate(constraints, context);
    }

    public static MotionPlanner boundedAngular(double period) {
        if (!Double.isFinite(period) || period <= 0.0) {
            throw new IllegalArgumentException("Angular period must be positive.");
        }
        return (context, constraints) -> {
            double current = context.current();
            double requested = context.requested();
            long center = Math.round((current - requested) / period);
            Double best = null;
            double bestDistance = Double.POSITIVE_INFINITY;
            for (long offset = -2; offset <= 2; offset++) {
                double candidate = requested + (center + offset) * period;
                ConstraintResult<Double> result = Constraints.evaluate(
                        constraints,
                        new ConstraintContext<>(current, candidate, context.runtime(), context.hardware()));
                if (!(result instanceof ConstraintResult.Allowed<Double>)) {
                    continue;
                }
                double distance = Math.abs(candidate - current);
                if (distance < bestDistance) {
                    best = candidate;
                    bestDistance = distance;
                }
            }
            return best == null
                    ? Constraints.evaluate(constraints, context)
                    : new ConstraintResult.Allowed<>(best);
        };
    }
}
