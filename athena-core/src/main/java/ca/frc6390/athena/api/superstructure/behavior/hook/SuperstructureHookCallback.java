package ca.frc6390.athena.api.superstructure.behavior.hook;

import java.util.function.Consumer;

import ca.frc6390.athena.mechanisms.SuperstructureContext;

@FunctionalInterface
public interface SuperstructureHookCallback<SP> extends Consumer<SuperstructureContext<SP>> {
    default void apply(SuperstructureContext<SP> context) {
        accept(context);
    }
}
