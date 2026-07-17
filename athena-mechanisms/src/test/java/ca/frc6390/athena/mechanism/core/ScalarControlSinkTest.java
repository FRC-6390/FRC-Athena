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
    void schedulerOwnsPrivateSinkBindingThroughItsPublicAction() {
        AimMechanism mechanism = new AimMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create()
                .register(mechanism)
                .bindInMemoryRuntime();

        scheduler.request(mechanism.aim).periodic(
                new MechanismContext(0.0, 0.0, 0.02, true, false, false),
                EventContext.empty());

        assertTrue(mechanism.angular.isActive());
        assertEquals(1.0, mechanism.angular.radiansPerSecond(), 1.0e-9);

        scheduler.cancel(mechanism.aim).periodic(
                new MechanismContext(0.02, 0.0, 0.02, true, false, false),
                EventContext.empty());
        assertFalse(mechanism.angular.isActive());
    }

    @Test
    void newestActionWinsSharedSinkAndPreviousActionResumes() {
        SharedSinkMechanism mechanism = new SharedSinkMechanism();
        MechanismScheduler scheduler = MechanismScheduler.create()
                .register(mechanism)
                .bindInMemoryRuntime();
        MechanismContext context = new MechanismContext(0.0, 0.0, 0.02, true, false, false);

        scheduler.request(mechanism.slow).periodic(context, EventContext.empty());
        assertEquals(1.0, mechanism.angular.radiansPerSecond(), 1.0e-9);

        scheduler.request(mechanism.fast).periodic(context, EventContext.empty());
        assertEquals(2.0, mechanism.angular.radiansPerSecond(), 1.0e-9);

        scheduler.cancel(mechanism.fast).periodic(context, EventContext.empty());
        assertEquals(1.0, mechanism.angular.radiansPerSecond(), 1.0e-9);
    }

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

    private static final class AimMechanism implements Mechanism {
        private final RobotVelocityPool pool = new RobotVelocityPool();
        private final RobotVelocityPool.AngularChannel angular = pool.angularChannel();
        private final ControlBinding heading = Controls.position(angular)
                .feedback(() -> 0.0)
                .pid(2.0, 0.0, 0.0)
                .constraint(Constraints.clamp(1.0));
        public final Action aim = heading.position(2.0);
    }

    private static final class SharedSinkMechanism implements Mechanism {
        private final RobotVelocityPool pool = new RobotVelocityPool();
        private final RobotVelocityPool.AngularChannel angular = pool.angularChannel();
        private final ControlBinding slowControl = Controls.position(angular)
                .feedback(() -> 0.0)
                .pid(1.0, 0.0, 0.0);
        private final ControlBinding fastControl = Controls.position(angular)
                .feedback(() -> 0.0)
                .pid(1.0, 0.0, 0.0);
        public final Action slow = slowControl.position(1.0);
        public final Action fast = fastControl.position(2.0);
    }
}
