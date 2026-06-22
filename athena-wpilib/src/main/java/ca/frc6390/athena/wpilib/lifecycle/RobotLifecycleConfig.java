package ca.frc6390.athena.wpilib.lifecycle;

import java.util.EnumMap;
import java.util.Map;

/**
 * Mutable robot lifecycle declaration.
 */
public final class RobotLifecycleConfig {
    private final Map<RobotMode, Runnable> initHooks = new EnumMap<>(RobotMode.class);
    private final Map<RobotMode, Runnable> periodicHooks = new EnumMap<>(RobotMode.class);

    /**
     * Registers an init hook for a robot mode.
     *
     * @param mode robot mode
     * @param hook hook callback
     * @return this config
     */
    public RobotLifecycleConfig onInit(RobotMode mode, Runnable hook) {
        initHooks.put(mode, hook == null ? () -> {
        } : hook);
        return this;
    }

    /**
     * Registers a periodic hook for a robot mode.
     *
     * @param mode robot mode
     * @param hook hook callback
     * @return this config
     */
    public RobotLifecycleConfig onPeriodic(RobotMode mode, Runnable hook) {
        periodicHooks.put(mode, hook == null ? () -> {
        } : hook);
        return this;
    }

    /**
     * Lowers this declaration into an immutable lifecycle spec.
     *
     * @return lifecycle spec
     */
    public RobotLifecycleSpec toSpec() {
        return new RobotLifecycleSpec(initHooks, periodicHooks);
    }
}
