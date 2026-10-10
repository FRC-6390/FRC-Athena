package ca.frc6390.athena.mechanism.effort;

/**
 * Implemented by control Actions that carry an {@link EffortRequest} (see {@code Action.effort(Measure)}).
 * The output path reads the request from the Action it is applying.
 */
public interface EffortCarrier {
    /**
     * Returns the effort request of this Action.
     *
     * @return request, {@link EffortRequest#NONE} when no effort was declared
     */
    EffortRequest effort();
}
