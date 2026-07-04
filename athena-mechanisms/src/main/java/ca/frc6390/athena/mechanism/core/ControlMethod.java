package ca.frc6390.athena.mechanism.core;

import java.util.List;
import java.util.Map;

/**
 * Open descriptor for the control algorithm used by a control binding.
 */
public interface ControlMethod {
    /**
     * Returns the method kind.
     *
     * @return method kind
     */
    String kind();

    /**
     * Returns numeric gains or parameters for this method.
     *
     * @return numeric parameters
     */
    default Map<String, Double> values() {
        return Map.of();
    }

    /**
     * Returns refs this method depends on.
     *
     * @return method refs
     */
    default List<Object> refs() {
        return List.of();
    }
}
