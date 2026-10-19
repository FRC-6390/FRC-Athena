package ca.frc6390.athena.mechanism.motion;

import ca.frc6390.athena.mechanism.constraint.ConstraintContext;
import ca.frc6390.athena.mechanism.constraint.ConstraintResult;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.measure.Angle;
import java.util.Objects;

/**
 * Factories for scalar motion planners.
 */
public final class MotionPlanners {
    private MotionPlanners() {
    }

    public static MotionPlanner direct() {
        return (context, constraints) -> Constraints.evaluate(constraints, context);
    }

    /**
     * Picks the equivalent angle ({@code requested + k * period}) closest to the current position
     * that the constraints accept unchanged.
     *
     * <p>Only candidates whose constraint result is {@link ConstraintResult.Allowed} qualify. A
     * {@link ConstraintResult.Corrected} result (for example a range clamp) disqualifies that wrap
     * candidate, because the corrected value is no longer the requested angle. When no candidate is
     * allowed unchanged, the planner falls back to evaluating the original request, which may then be
     * corrected (clamped) or rejected.</p>
     *
     * @param periodAngle wrap period, positive
     * @param unit angle unit of the binding's values; the period is converted to it once, here
     * @return planner
     */
    public static MotionPlanner boundedAngular(Angle periodAngle, AngleUnit unit) {
        Objects.requireNonNull(periodAngle, "periodAngle");
        Objects.requireNonNull(unit, "unit");
        double period = periodAngle.in(unit);
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
                        new ConstraintContext<>(current, candidate, context.runtime()));
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
