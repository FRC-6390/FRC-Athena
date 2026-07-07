package ca.frc6390.athena.mechanism.core;

import java.util.List;

/**
 * Typed control-loop declaration that can create its runtime logic.
 */
public interface ControlLoopRef {
    /**
     * Creates runtime loop state for one bound control.
     *
     * @param binding control binding
     * @return runtime loop
     */
    ControlLoopRuntime bind(ControlLoopBinding binding);

    /**
     * Returns refs this loop depends on.
     *
     * @return dependent refs
     */
    default List<Object> refs() {
        return List.of();
    }
}
