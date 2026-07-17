package frc.robot.vision;

import ca.frc6390.athena.mechanism.core.Action;
import ca.frc6390.athena.mechanism.core.Actions;
import ca.frc6390.athena.mechanism.core.Mechanism;
import ca.frc6390.athena.mechanism.core.Telemetry;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.measurement.TargetMeasurementSample;
import ca.frc6390.athena.vision.signal.TargetSignal;
import frc.robot.DriveTrain;

/** Camera-target reads converted into live drivetrain Actions. */
public final class TargetingExamples implements Mechanism {
    private static final double AIM_KP = 0.035;
    private static final double RANGE_KP = 0.8;
    private final DriveTrain drive;
    public final TargetSignal targets;

    public final Action stop;
    public final Action aim;
    public final Action approach;

    public TargetingExamples(DriveTrain drive, VisionSources vision) {
        this.drive = drive;
        targets = vision.limelightTargets;
        stop = Actions.compute(() -> {
            drive.targetingVelocity.clear();
            return drive.pooledDrive;
        });
        aim = Actions.compute(() -> {
            drive.targetingVelocity.set(targets.latest()
                    .map(target -> RobotVelocity.angular(
                            clamp(-target.yawDegrees() * AIM_KP, 1.5)))
                    .orElseGet(RobotVelocity::zero));
            return drive.pooledDrive;
        });
        approach = Actions.compute(() -> {
            drive.targetingVelocity.set(targets.latest()
                    .map(target -> RobotVelocity.robot(
                            clamp((target.distanceMeters() - 1.5) * RANGE_KP, 2.0),
                            0.0,
                            clamp(-target.yawDegrees() * AIM_KP, 1.5)))
                    .orElseGet(RobotVelocity::zero));
            return drive.pooledDrive;
        });
    }

    @Telemetry("hasTarget")
    public boolean hasTarget() {
        return targets.hasTarget();
    }

    @Telemetry("targetYawDegrees")
    public double targetYawDegrees() {
        return targets.latest().map(TargetMeasurementSample::yawDegrees).orElse(Double.NaN);
    }

    @Telemetry("targetDistanceMeters")
    public double targetDistanceMeters() {
        return targets.latest().map(TargetMeasurementSample::distanceMeters).orElse(Double.NaN);
    }

    private static double clamp(double value, double magnitude) {
        return Math.max(-magnitude, Math.min(magnitude, value));
    }
}
