package ca.frc6390.athena.localization.ref;

import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;

record PoseSample(
        PoseSnapshot pose,
        RobotVelocity speeds,
        double timestampSeconds,
        double latencySeconds,
        int targetCount,
        MeasurementStdDevs stdDevs) {
}
