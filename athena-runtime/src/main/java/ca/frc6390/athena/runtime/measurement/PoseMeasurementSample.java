package ca.frc6390.athena.runtime.measurement;

import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;

/**
 * Typed pose-capable measurement sample.
 */
public interface PoseMeasurementSample extends Measurement {
    /**
     * Returns the measured field-relative pose.
     *
     * @return pose
     */
    PoseSnapshot pose();

    /**
     * Returns the measured robot velocity.
     *
     * @return robot velocity
     */
    RobotVelocity speeds();

    /**
     * Returns source ambiguity where lower is better.
     *
     * @return ambiguity
     */
    double ambiguity();

    /**
     * Returns the number of targets used by this pose estimate.
     *
     * @return target count
     */
    int targetCount();

    /**
     * Returns measurement standard deviations.
     *
     * @return standard deviations
     */
    MeasurementStdDevs stdDevs();
}
