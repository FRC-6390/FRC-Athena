package ca.frc6390.athena.api.mechanism.behavior.automation;

import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.mechanisms.MechanismContext;

@FunctionalInterface
public interface MechanismStateHookCallback {
    void apply(MechanismContext<? extends Mechanism, ?> context);
}
