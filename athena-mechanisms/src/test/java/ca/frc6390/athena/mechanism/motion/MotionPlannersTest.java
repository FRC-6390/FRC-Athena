package ca.frc6390.athena.mechanism.motion;

import static org.junit.jupiter.api.Assertions.assertEquals;

import ca.frc6390.athena.hardware.device.Range;
import ca.frc6390.athena.mechanism.constraint.ConstraintContext;
import ca.frc6390.athena.mechanism.constraint.ConstraintResult;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.mechanism.core.MechanismContext;
import java.util.List;
import org.junit.jupiter.api.Test;

class MotionPlannersTest {
    @Test
    void boundedAngularChoosesNearestEquivalentInsideConstraints() {
        ConstraintResult<Double> result = MotionPlanners.boundedAngular(360.0).plan(
                new ConstraintContext<>(250.0, -250.0, MechanismContext.empty()),
                List.of(Constraints.range(Range.degrees(-270.0, 270.0))));

        assertEquals(110.0, result.value(), 1.0e-9);
    }
}
