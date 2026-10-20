package ca.frc6390.athena.runtime.measurement;

import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import java.util.Optional;

import static edu.wpi.first.units.Units.Meters;

/** A stream of field-pose measurements that can feed localization stages. */
public interface PoseSignal extends MeasurementSignal {
    /** Returns the newest typed pose measurement, if one is available. */
    default Optional<PoseMeasurementSample> measurement() {
        // The first of equally new samples wins, as with Stream.max.
        PoseMeasurementSample newest = null;
        for (Measurement measurement : measurements()) {
            if (measurement instanceof PoseMeasurementSample sample
                    && (newest == null
                            || Double.compare(sample.timestampSeconds(), newest.timestampSeconds()) > 0)) {
                newest = sample;
            }
        }
        return Optional.ofNullable(newest);
    }

    /** Returns the newest pose value, if one is available. */
    default Optional<Pose2d> value() {
        return measurement().map(sample -> PoseSignals.toPose2d(sample.pose()));
    }

    /** Returns the newest pose, or the origin when no pose is available. */
    default Pose2d pose() {
        return PoseSignals.toPose2d(poseSnapshot());
    }

    /**
     * Returns the newest pose in the runtime's internal snapshot form, or the origin when none is
     * available. Framework code uses this; robot code uses {@link #pose()}.
     */
    default PoseSnapshot poseSnapshot() {
        return measurement().map(PoseMeasurementSample::pose).orElseGet(() -> new PoseSnapshot(0.0, 0.0, 0.0));
    }

    /** Returns speeds from the newest pose sample. */
    default RobotVelocity speeds() {
        return measurement()
                .map(PoseMeasurementSample::speeds)
                .orElseGet(RobotVelocity::zero);
    }

    /** Applies covariance to single-tag samples from this source. */
    default PoseSignal singleTagStdDevs(Distance xStdDev, Distance yStdDev, Angle headingStdDev) {
        return PoseSignals.tagStdDevs(this, false, MeasurementStdDevs.of(xStdDev, yStdDev, headingStdDev));
    }

    /** Applies covariance to multi-tag samples from this source. */
    default PoseSignal multiTagStdDevs(Distance xStdDev, Distance yStdDev, Angle headingStdDev) {
        return PoseSignals.tagStdDevs(this, true, MeasurementStdDevs.of(xStdDev, yStdDev, headingStdDev));
    }

    /** Scales standard deviations by max(1, (average target distance / reference)^exponent). */
    default PoseSignal distanceStdDevScaling(Distance reference, double exponent) {
        return PoseSignals.distanceStdDevScaling(this, reference.in(Meters), exponent);
    }

    /**
     * Uses this source for field translation while leaving heading to odometry/IMU inputs.
     *
     * @return translation-only pose signal
     */
    default PoseSignal translationOnly() {
        return PoseSignals.translationOnly(this);
    }

    /**
     * Uses this source for field heading while leaving translation to odometry or other pose inputs.
     *
     * @return heading-only pose signal
     */
    default PoseSignal headingOnly() {
        return PoseSignals.headingOnly(this);
    }
}
