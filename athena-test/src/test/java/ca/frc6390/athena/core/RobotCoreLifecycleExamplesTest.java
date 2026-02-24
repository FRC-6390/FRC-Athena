package ca.frc6390.athena.core;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

import ca.frc6390.athena.core.examples.RobotCoreLifecycleExamples;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import org.junit.jupiter.api.Test;

final class RobotCoreLifecycleExamplesTest {

    @Test
    void phaseDispatchRunsExpectedBindingsInOrder() {
        List<String> timeline = new ArrayList<>();
        RobotCoreHooks<SwerveDrivetrain> hooks = RobotCoreLifecycleExamples.createLifecycleHooks(timeline);

        RobotCoreLifecycleExamples.runPhase(hooks, RobotCoreHooks.Phase.ROBOT_INIT, true);
        RobotCoreLifecycleExamples.runPhase(hooks, RobotCoreHooks.Phase.TELEOP_INIT, true);
        RobotCoreLifecycleExamples.runPhase(hooks, RobotCoreHooks.Phase.TELEOP_PERIODIC, true);

        assertEquals(List.of(
                "robot.init",
                "teleop.init",
                "teleop.periodic",
                "teleop.periodic.loop"), timeline);
    }

    @Test
    void periodicLoopBindingsAreScopedPerPhase() {
        RobotCoreHooks<SwerveDrivetrain> hooks =
                RobotCoreLifecycleExamples.createLifecycleHooks(new ArrayList<>());

        Map<RobotCoreHooks.Phase, List<RobotCoreHooks.PeriodicHookBinding<SwerveDrivetrain>>> byPhase =
                RobotCoreLifecycleExamples.periodicLoopBindingsByPhase(hooks);

        assertTrue(byPhase.containsKey(RobotCoreHooks.Phase.TEST_PERIODIC));
        assertEquals(1, byPhase.get(RobotCoreHooks.Phase.TEST_PERIODIC).size());
        assertEquals(20.0, byPhase.get(RobotCoreHooks.Phase.TEST_PERIODIC).get(0).periodMs(), 1e-9);
        assertFalse(byPhase.containsKey(RobotCoreHooks.Phase.TELEOP_INIT));
    }

    @Test
    void nonPeriodicPhasesDoNotRunLoopBindings() {
        List<String> timeline = new ArrayList<>();
        RobotCoreHooks<SwerveDrivetrain> hooks = RobotCoreLifecycleExamples.createLifecycleHooks(timeline);

        RobotCoreLifecycleExamples.runPhase(hooks, RobotCoreHooks.Phase.TEST_INIT, true);

        assertEquals(List.of("test.init"), timeline);
    }
}
