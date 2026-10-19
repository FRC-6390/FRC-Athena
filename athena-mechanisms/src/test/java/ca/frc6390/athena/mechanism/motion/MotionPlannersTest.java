package ca.frc6390.athena.mechanism.motion;

import edu.wpi.first.units.Units;
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
                List.of(Constraints.range(Range.of(Units.Degrees.of(-270.0), Units.Degrees.of(270.0)))));

        assertEquals(110.0, result.value(), 1.0e-9);
    }
}
