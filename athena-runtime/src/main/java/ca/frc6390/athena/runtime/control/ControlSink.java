package ca.frc6390.athena.runtime.control;

import java.util.List;

/** Destination for one scalar control-loop result. */
public interface ControlSink {
    /** Applies the latest calculated control value. */
    void apply(double value);

    /** Releases this destination when its action is no longer active. */
    void release();

    /** Returns declarations required by this destination. */
    default List<?> dependencies() {
        return List.of();
    }
}
