package ca.frc6390.athena.core.hooks;

import java.util.ArrayList;
import java.util.List;

import ca.frc6390.athena.core.RobotCoreHooks;

/**
 * Shared phase-hook aliases for hook sections that support optional state filters.
 */
public abstract class LifecycleHooksSectionBase<Self, B, S> {

    protected abstract Self self();

    protected abstract void addPhaseBinding(RobotCoreHooks.Phase phase, B binding, List<S> states);

    protected abstract void addPhaseExitBinding(B binding, List<S> states);

    @SafeVarargs
    public final Self onInit(B binding, S... states) {
        addPhaseBinding(RobotCoreHooks.Phase.ROBOT_INIT, binding, copyStates(states));
        return self();
    }

    @SafeVarargs
    public final Self onPeriodic(B binding, S... states) {
        addPhaseBinding(RobotCoreHooks.Phase.ROBOT_PERIODIC, binding, copyStates(states));
        return self();
    }

    @SafeVarargs
    public final Self onExit(B binding, S... states) {
        addPhaseExitBinding(binding, copyStates(states));
        return self();
    }

    @SafeVarargs
    public final Self onDisabledInit(B binding, S... states) {
        addPhaseBinding(RobotCoreHooks.Phase.DISABLED_INIT, binding, copyStates(states));
        return self();
    }

    @SafeVarargs
    public final Self onDisabledPeriodic(B binding, S... states) {
        addPhaseBinding(RobotCoreHooks.Phase.DISABLED_PERIODIC, binding, copyStates(states));
        return self();
    }

    @SafeVarargs
    public final Self onDisabledExit(B binding, S... states) {
        addPhaseBinding(RobotCoreHooks.Phase.DISABLED_EXIT, binding, copyStates(states));
        return self();
    }

    @SafeVarargs
    public final Self onTeleopInit(B binding, S... states) {
        addPhaseBinding(RobotCoreHooks.Phase.TELEOP_INIT, binding, copyStates(states));
        return self();
    }

    @SafeVarargs
    public final Self onTeleInit(B binding, S... states) {
        return onTeleopInit(binding, states);
    }

    @SafeVarargs
    public final Self onTeleopPeriodic(B binding, S... states) {
        addPhaseBinding(RobotCoreHooks.Phase.TELEOP_PERIODIC, binding, copyStates(states));
        return self();
    }

    @SafeVarargs
    public final Self onTelePeriodic(B binding, S... states) {
        return onTeleopPeriodic(binding, states);
    }

    @SafeVarargs
    public final Self onTeleopExit(B binding, S... states) {
        addPhaseBinding(RobotCoreHooks.Phase.TELEOP_EXIT, binding, copyStates(states));
        return self();
    }

    @SafeVarargs
    public final Self onTeleExit(B binding, S... states) {
        return onTeleopExit(binding, states);
    }

    @SafeVarargs
    public final Self onAutonomousInit(B binding, S... states) {
        addPhaseBinding(RobotCoreHooks.Phase.AUTONOMOUS_INIT, binding, copyStates(states));
        return self();
    }

    @SafeVarargs
    public final Self onAutoInit(B binding, S... states) {
        return onAutonomousInit(binding, states);
    }

    @SafeVarargs
    public final Self onAutonomousPeriodic(B binding, S... states) {
        addPhaseBinding(RobotCoreHooks.Phase.AUTONOMOUS_PERIODIC, binding, copyStates(states));
        return self();
    }

    @SafeVarargs
    public final Self onAutoPeriodic(B binding, S... states) {
        return onAutonomousPeriodic(binding, states);
    }

    @SafeVarargs
    public final Self onAutonomousExit(B binding, S... states) {
        addPhaseBinding(RobotCoreHooks.Phase.AUTONOMOUS_EXIT, binding, copyStates(states));
        return self();
    }

    @SafeVarargs
    public final Self onAutoExit(B binding, S... states) {
        return onAutonomousExit(binding, states);
    }

    @SafeVarargs
    public final Self onTestInit(B binding, S... states) {
        addPhaseBinding(RobotCoreHooks.Phase.TEST_INIT, binding, copyStates(states));
        return self();
    }

    @SafeVarargs
    public final Self onTestPeriodic(B binding, S... states) {
        addPhaseBinding(RobotCoreHooks.Phase.TEST_PERIODIC, binding, copyStates(states));
        return self();
    }

    @SafeVarargs
    public final Self onTestExit(B binding, S... states) {
        addPhaseBinding(RobotCoreHooks.Phase.TEST_EXIT, binding, copyStates(states));
        return self();
    }

    @SafeVarargs
    private final List<S> copyStates(S... states) {
        if (states == null || states.length == 0) {
            return List.of();
        }
        List<S> copied = new ArrayList<>(states.length);
        for (S state : states) {
            if (state != null) {
                copied.add(state);
            }
        }
        return copied;
    }
}
