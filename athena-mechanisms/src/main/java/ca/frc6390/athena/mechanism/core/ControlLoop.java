package ca.frc6390.athena.mechanism.core;

import java.util.List;

/**
 * Typed control-loop declaration that can create its runtime logic.
 */
public interface ControlLoop {
    /**
     * Creates runtime loop Action for one bound control.
     *
     * @param binding control binding
     * @return runtime loop
     */
    ControlLoopRuntime bind(ControlLoopBinding binding);

    /**
     * Returns declarations this loop depends on.
     *
     * @return dependent declarations
     */
    default List<Object> dependencies() {
        return List.of();
    }
}
