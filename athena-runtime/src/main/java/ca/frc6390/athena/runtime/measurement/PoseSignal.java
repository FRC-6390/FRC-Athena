package ca.frc6390.athena.runtime.measurement;

import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import java.util.Comparator;
import java.util.Optional;

/** A stream of field-pose measurements that can feed localization stages. */
public interface PoseSignal extends MeasurementSignal {
    /** Returns the newest typed pose measurement, if one is available. */
    default Optional<PoseMeasurementSample> measurement() {
        return measurements().stream()
                .filter(PoseMeasurementSample.class::isInstance)
                .map(PoseMeasurementSample.class::cast)
                .max(Comparator.comparingDouble(PoseMeasurementSample::timestampSeconds));
    }

    /** Returns the newest pose value, if one is available. */
    default Optional<PoseSnapshot> value() {
        return measurement().map(PoseMeasurementSample::pose);
    }

    /** Returns the newest pose, or the origin when no pose is available. */
    default PoseSnapshot pose() {
        return value()
                .orElseGet(() -> new PoseSnapshot(0.0, 0.0, 0.0));
    }

    /** Returns speeds from the newest pose sample. */
    default RobotVelocity speeds() {
        return measurement()
                .map(PoseMeasurementSample::speeds)
                .orElseGet(RobotVelocity::zero);
    }

    /** Applies covariance to single-tag samples from this source. */
    default PoseSignal singleTagStdDevs(double xMeters, double yMeters, double headingRadians) {
        return PoseSignals.tagStdDevs(this, false, MeasurementStdDevs.of(xMeters, yMeters, headingRadians));
    }

    /** Applies covariance to multi-tag samples from this source. */
    default PoseSignal multiTagStdDevs(double xMeters, double yMeters, double headingRadians) {
        return PoseSignals.tagStdDevs(this, true, MeasurementStdDevs.of(xMeters, yMeters, headingRadians));
    }

    /** Scales standard deviations by max(1, (average target distance / reference)^exponent). */
    default PoseSignal distanceStdDevScaling(double referenceMeters, double exponent) {
        return PoseSignals.distanceStdDevScaling(this, referenceMeters, exponent);
    }
}
