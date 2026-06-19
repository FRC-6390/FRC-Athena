package ca.frc6390.athena.api.superstructure.behavior.hook;

import ca.frc6390.athena.mechanisms.SuperstructureContext;

@FunctionalInterface
public interface SuperstructureTransitionHookCallback<SP> {
    void apply(SuperstructureContext<SP> context, Object from, Object to);
}
