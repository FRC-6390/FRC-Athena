package ca.frc6390.athena.wpilib.lifecycle;

import java.util.EnumMap;
import java.util.Map;

/**
 * Immutable robot lifecycle hooks.
 *
 * @param initHooks hooks run when a mode starts
 * @param periodicHooks hooks run during a mode's periodic loop
 */
public record RobotLifecycleSpec(
        Map<RobotMode, Runnable> initHooks,
        Map<RobotMode, Runnable> periodicHooks) {
    public RobotLifecycleSpec {
        initHooks = copy(initHooks);
        periodicHooks = copy(periodicHooks);
    }

    /**
     * Runs the init hook for a mode when present.
     *
     * @param mode robot mode
     */
    public void runInit(RobotMode mode) {
        initHooks.getOrDefault(mode, () -> {
        }).run();
    }

    /**
     * Runs the periodic hook for a mode when present.
     *
     * @param mode robot mode
     */
    public void runPeriodic(RobotMode mode) {
        periodicHooks.getOrDefault(mode, () -> {
        }).run();
    }

    private static Map<RobotMode, Runnable> copy(Map<RobotMode, Runnable> hooks) {
        EnumMap<RobotMode, Runnable> copy = new EnumMap<>(RobotMode.class);
        if (hooks != null) {
            copy.putAll(hooks);
        }
        return Map.copyOf(copy);
    }
}
