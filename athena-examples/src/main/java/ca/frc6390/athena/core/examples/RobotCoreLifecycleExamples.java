package ca.frc6390.athena.core.examples;

import java.util.List;
import java.util.Map;

import ca.frc6390.athena.core.RobotCoreHooks;
import ca.frc6390.athena.core.RobotDrivetrain;

/**
 * Example helpers for lifecycle-phase hook registration and deterministic phase dispatch.
 */
public final class RobotCoreLifecycleExamples {
    private RobotCoreLifecycleExamples() {}

    public static <T extends RobotDrivetrain<T>> RobotCoreHooks<T> createLifecycleHooks(List<String> timeline) {
        return RobotCoreHooks.<T>empty().hooks(hooks -> {
            hooks.onInit(ctx -> timeline.add("robot.init"));
            hooks.onPeriodic(ctx -> timeline.add("robot.periodic"));
            hooks.onDisabledInit(ctx -> timeline.add("disabled.init"));
            hooks.onDisabledPeriodic(ctx -> timeline.add("disabled.periodic"));
            hooks.onTeleopInit(ctx -> timeline.add("teleop.init"));
            hooks.onTeleopPeriodic(ctx -> timeline.add("teleop.periodic"));
            hooks.onAutonomousInit(ctx -> timeline.add("autonomous.init"));
            hooks.onAutonomousPeriodic(ctx -> timeline.add("autonomous.periodic"));
            hooks.onTestInit(ctx -> timeline.add("test.init"));
            hooks.onTestPeriodic(ctx -> timeline.add("test.periodic"));

            hooks.onPeriodic(ctx -> timeline.add("robot.periodic.loop"), 20.0);
            hooks.onTeleopPeriodic(ctx -> timeline.add("teleop.periodic.loop"), 20.0);
            hooks.onAutonomousPeriodic(ctx -> timeline.add("autonomous.periodic.loop"), 20.0);
            hooks.onDisabledPeriodic(ctx -> timeline.add("disabled.periodic.loop"), 20.0);
            hooks.onTestPeriodic(ctx -> timeline.add("test.periodic.loop"), 20.0);
        });
    }

    public static <T extends RobotDrivetrain<T>> List<RobotCoreHooks.Binding<T>> bindingsForPhase(
            RobotCoreHooks<T> hooks,
            RobotCoreHooks.Phase phase) {
        return switch (phase) {
            case ROBOT_INIT -> hooks.initBindings();
            case ROBOT_PERIODIC -> hooks.periodicBindings();
            case DISABLED_INIT -> hooks.disabledInitBindings();
            case DISABLED_PERIODIC -> hooks.disabledPeriodicBindings();
            case DISABLED_EXIT -> hooks.disabledExitBindings();
            case TELEOP_INIT -> hooks.teleopInitBindings();
            case TELEOP_PERIODIC -> hooks.teleopPeriodicBindings();
            case TELEOP_EXIT -> hooks.teleopExitBindings();
            case AUTONOMOUS_INIT -> hooks.autonomousInitBindings();
            case AUTONOMOUS_PERIODIC -> hooks.autonomousPeriodicBindings();
            case AUTONOMOUS_EXIT -> hooks.autonomousExitBindings();
            case TEST_INIT -> hooks.testInitBindings();
            case TEST_PERIODIC -> hooks.testPeriodicBindings();
            case TEST_EXIT -> hooks.testExitBindings();
        };
    }

    public static <T extends RobotDrivetrain<T>> Map<RobotCoreHooks.Phase, List<RobotCoreHooks.PeriodicHookBinding<T>>>
            periodicLoopBindingsByPhase(RobotCoreHooks<T> hooks) {
        return Map.of(
                RobotCoreHooks.Phase.ROBOT_PERIODIC, hooks.periodicLoopBindings(),
                RobotCoreHooks.Phase.DISABLED_PERIODIC, hooks.disabledPeriodicLoopBindings(),
                RobotCoreHooks.Phase.TELEOP_PERIODIC, hooks.teleopPeriodicLoopBindings(),
                RobotCoreHooks.Phase.AUTONOMOUS_PERIODIC, hooks.autonomousPeriodicLoopBindings(),
                RobotCoreHooks.Phase.TEST_PERIODIC, hooks.testPeriodicLoopBindings());
    }

    public static <T extends RobotDrivetrain<T>> void runPhase(
            RobotCoreHooks<T> hooks,
            RobotCoreHooks.Phase phase,
            boolean includePeriodicLoops) {
        for (RobotCoreHooks.Binding<T> binding : bindingsForPhase(hooks, phase)) {
            binding.apply(null);
        }
        if (!includePeriodicLoops) {
            return;
        }
        List<RobotCoreHooks.PeriodicHookBinding<T>> loops = periodicLoopBindingsByPhase(hooks).get(phase);
        if (loops == null) {
            return;
        }
        for (RobotCoreHooks.PeriodicHookBinding<T> loop : loops) {
            loop.hook().apply(null);
        }
    }
}
