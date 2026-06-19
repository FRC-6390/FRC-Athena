package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.core.RobotCoreHooks;

interface SuperstructureLifecycleHooks<S, SP> {
    enum NoState {
        NONE
    }

    SuperstructureLifecycleHooks<?, ?> NONE = new SuperstructureLifecycleHooks<NoState, Object>() {
    };

    static <S, SP> SuperstructureLifecycleHooks<S, SP> from(SuperstructureRuntimeConfig<S, SP> config) {
        if (config == null) {
            return none();
        }
        return new SuperstructureLifecycleHooks<>() {
            @Override
            public void runInitHooks(SuperstructureContext<SP> context, S state) {
                runPhaseHooks(context, state, RobotCoreHooks.Phase.ROBOT_INIT);
            }

            @Override
            public void runPhaseHooks(SuperstructureContext<SP> context, S state, RobotCoreHooks.Phase phase) {
                if (context == null || phase == null) {
                    return;
                }
                if (isExitPhase(phase)) {
                    runLifecycleHooks(context, state, config.lifecycleExitBindings());
                }
                runLifecycleHooks(context, state, config.lifecycleBindings().get(phase));
            }
        };
    }

    @SuppressWarnings("unchecked")
    static <S, SP> SuperstructureLifecycleHooks<S, SP> none() {
        return (SuperstructureLifecycleHooks<S, SP>) NONE;
    }

    default void runInitHooks(SuperstructureContext<SP> context, S state) {
    }

    default void runPhaseHooks(SuperstructureContext<SP> context, S state, RobotCoreHooks.Phase phase) {
    }

    private static <S, SP> void runLifecycleHooks(
            SuperstructureContext<SP> context,
            S activeState,
            java.util.List<SuperstructureRuntimeConfig.LifecycleHookBinding<SP, S>> bindings) {
        if (bindings == null || bindings.isEmpty()) {
            return;
        }
        for (SuperstructureRuntimeConfig.LifecycleHookBinding<SP, S> hook : bindings) {
            if (hook == null || hook.binding() == null || !hook.appliesTo(activeState)) {
                continue;
            }
            hook.binding().apply(context);
        }
    }

    private static boolean isExitPhase(RobotCoreHooks.Phase phase) {
        return phase == RobotCoreHooks.Phase.DISABLED_EXIT
                || phase == RobotCoreHooks.Phase.TELEOP_EXIT
                || phase == RobotCoreHooks.Phase.AUTONOMOUS_EXIT
                || phase == RobotCoreHooks.Phase.TEST_EXIT;
    }
}
