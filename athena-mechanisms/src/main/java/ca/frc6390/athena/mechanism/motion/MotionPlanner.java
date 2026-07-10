package ca.frc6390.athena.mechanism.motion;

import ca.frc6390.athena.mechanism.constraint.Constraint;
import ca.frc6390.athena.mechanism.constraint.ConstraintContext;
import ca.frc6390.athena.mechanism.constraint.ConstraintResult;
import java.util.List;

/**
 * Selects a legal scalar position target before profiling begins.
 */
@FunctionalInterface
public interface MotionPlanner {
    ConstraintResult<Double> plan(
            ConstraintContext<Double> context,
            List<Constraint<Double>> constraints);
}
