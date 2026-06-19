package ca.frc6390.athena.mechanisms;

import ca.frc6390.athena.core.RobotCoreHooks;

interface MechanismLifecycleHooks {
    MechanismLifecycleHooks NONE = new MechanismLifecycleHooks() {
    };

    default void runInitHooks(Mechanism mechanism) {
    }

    default void runPhaseHooks(Mechanism mechanism, RobotCoreHooks.Phase phase) {
    }
}
