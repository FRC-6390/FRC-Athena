package ca.frc6390.athena.mechanism.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.mechanism.constraint.Constraints;
import ca.frc6390.athena.runtime.control.RobotVelocityPool;
import org.junit.jupiter.api.Test;

class ScalarControlSinkTest {
    @Test
    void pidOutputIsClampedAndAppliedToAngularChannel() {
        RobotVelocityPool pool = new RobotVelocityPool();
        RobotVelocityPool.AngularChannel angular = pool.angularChannel();
        ControlBinding control = Controls.position(angular)
                .feedback(() -> 0.0, () -> 0.0)
                .pid(2.0, 0.0, 0.0)
                .constraint(Constraints.clamp(1.5));
        Output target = Outputs.position(4.0);
        OutputApplier applier = OutputApplier.using(ActionContext.empty());

        applier.beginCycle();
        applier.apply(new ResolvedOutput(OutputRequest.of(control, target), target));
        applier.endCycle();

        assertTrue(angular.isActive());
        assertEquals(1.5, angular.radiansPerSecond(), 1.0e-9);

        applier.beginCycle();
        applier.endCycle();
        assertFalse(angular.isActive());
    }

    @Test
    void continuousPositionUsesShortestWrappedError() {
        double measurement = Math.toRadians(179.0);
        RobotVelocityPool.AngularChannel angular = new RobotVelocityPool().angularChannel();
        ControlBinding control = Controls.position(angular)
                .feedback(() -> measurement, () -> 0.0)
                .pid(1.0, 0.0, 0.0)
                .continuous(-Math.PI, Math.PI);
        Output target = Outputs.position(Math.toRadians(-179.0));

        OutputApplier.using(ActionContext.empty())
                .apply(new ResolvedOutput(OutputRequest.of(control, target), target));

        assertEquals(Math.toRadians(2.0), angular.radiansPerSecond(), 1.0e-9);
    }

    @Test
    void outputOnlyClampDoesNotRequireFeedback() {
        RobotVelocityPool.AngularChannel angular = new RobotVelocityPool().angularChannel();
        ControlBinding control = Controls.position(angular)
                .constraint(Constraints.clamp(0.75));
        Output target = Outputs.position(2.0);

        OutputApplier.using(ActionContext.empty())
                .apply(new ResolvedOutput(OutputRequest.of(control, target), target));

        assertEquals(0.75, angular.radiansPerSecond(), 1.0e-9);
    }
}
