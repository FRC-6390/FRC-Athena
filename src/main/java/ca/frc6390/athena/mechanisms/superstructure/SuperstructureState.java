package ca.frc6390.athena.mechanisms.superstructure;

import ca.frc6390.athena.mechanisms.StateMachine.SetpointProvider;

/**
 * Contract for superstructure macro states. Teams typically implement this interface on an enum that
 * represents high-level poses or routines. The enum receives a {@link SuperstructureContext} during
 * configuration so each state can declare the mechanisms it drives, guard conditions, and any side
 * effects required while running.
 *
 * @param <C> context type shared across macro definitions (e.g. references to mechanism handles,
 *            vision helpers, LED controllers, etc.)
 */
public interface SuperstructureState<C extends SuperstructureContext>
        extends SetpointProvider<SuperstructureStateDef> {

    /**
     * Called by {@link SuperstructureConfig} while building the superstructure. Implementations
     * describe their steps, completion guards, and side effects by mutating the supplied builder.
     */
    void configure(SuperstructureStateDef.Builder builder, C context);

    @Override
    default SuperstructureStateDef getSetpoint() {
        return SuperstructureStateRegistry.get((Enum<?>) this);
    }
}
