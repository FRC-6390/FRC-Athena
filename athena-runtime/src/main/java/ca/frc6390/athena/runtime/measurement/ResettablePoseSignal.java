package ca.frc6390.athena.runtime.measurement;

import ca.frc6390.athena.runtime.filter.PoseSnapshot;

/** Measurement source whose tracked field pose can be reset. */
public interface ResettablePoseSignal extends MeasurementSignal {
    /** Resets the tracked field pose. */
    void reset(PoseSnapshot pose);
}
