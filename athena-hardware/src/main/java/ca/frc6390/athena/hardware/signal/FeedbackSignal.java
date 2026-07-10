package ca.frc6390.athena.hardware.signal;

import java.util.List;

/**
 * Declarative input that can participate in control feedback composition.
 */
public interface FeedbackSignal {
    /**
     * Returns the physical declarations needed to evaluate this signal.
     *
     * @return signal dependencies
     */
    default List<?> dependencies() {
        return List.of();
    }
}
