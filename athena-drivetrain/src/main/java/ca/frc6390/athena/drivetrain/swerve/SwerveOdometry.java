package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.HardwareMeasurementSignal;
import ca.frc6390.athena.hardware.signal.ImuSource;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.runtime.measurement.ResettablePoseSignal;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/** Runtime-refreshed odometry derived from measured swerve module travel and IMU heading. */
public final class SwerveOdometry implements HardwareMeasurementSignal, ResettablePoseSignal {
    private static final double TWO_PI = Math.PI * 2.0;

    private final SwerveKinematics kinematics;
    private final ImuSource imu;
    private final List<EncoderDevice> driveEncoders;
    private double[] previousDistances;
    private PoseSnapshot pose = new PoseSnapshot(0.0, 0.0, 0.0);
    private PoseSnapshot pendingReset;
    private RobotVelocity velocity = RobotVelocity.zero();
    private double headingOffsetRadians;
    private double previousHeadingRadians;
    private Measurement measurement;

    SwerveOdometry(SwerveKinematics kinematics, ImuSource imu) {
        this.kinematics = Objects.requireNonNull(kinematics, "kinematics");
        this.imu = Objects.requireNonNull(imu, "imu");
        driveEncoders = kinematics.modules().stream()
                .map(positioned -> distanceEncoder(positioned.module()))
                .toList();
    }

    @Override
    public synchronized void refresh(ActionContext context, double timestampSeconds, double dtSeconds) {
        Objects.requireNonNull(context, "context");
        double[] distances = new double[driveEncoders.size()];
        List<Double> angles = new ArrayList<>(driveEncoders.size());
        for (int index = 0; index < driveEncoders.size(); index++) {
            distances[index] = driveEncoders.get(index).position(context);
            angles.add(kinematics.modules().get(index).module().angle.get().absolutePosition().position(context));
        }

        double rawHeading = Math.toRadians(imu.yawDegrees());
        if (pendingReset != null) {
            pose = pendingReset;
            headingOffsetRadians = pose.headingRadians() - rawHeading;
            pendingReset = null;
            previousDistances = distances;
            previousHeadingRadians = pose.headingRadians();
            velocity = RobotVelocity.zero();
        } else if (previousDistances == null) {
            headingOffsetRadians = pose.headingRadians() - rawHeading;
            previousDistances = distances;
            previousHeadingRadians = pose.headingRadians();
        } else {
            double seconds = Double.isFinite(dtSeconds) && dtSeconds > 0.0 ? dtSeconds : 0.02;
            List<SwerveModuleTarget> measuredStates = new ArrayList<>(distances.length);
            for (int index = 0; index < distances.length; index++) {
                measuredStates.add(new SwerveModuleTarget(
                        (distances[index] - previousDistances[index]) / seconds,
                        angles.get(index)));
            }
            RobotVelocity wheelVelocity = kinematics.velocity(measuredStates);
            double heading = rawHeading + headingOffsetRadians;
            double headingDelta = wrapRadians(heading - previousHeadingRadians);
            double midpointHeading = previousHeadingRadians + headingDelta / 2.0;
            double robotDx = wheelVelocity.xMetersPerSecond() * seconds;
            double robotDy = wheelVelocity.yMetersPerSecond() * seconds;
            double cos = Math.cos(midpointHeading);
            double sin = Math.sin(midpointHeading);
            pose = new PoseSnapshot(
                    pose.xMeters() + robotDx * cos - robotDy * sin,
                    pose.yMeters() + robotDx * sin + robotDy * cos,
                    heading);
            velocity = new RobotVelocity(
                    wheelVelocity.xMetersPerSecond(),
                    wheelVelocity.yMetersPerSecond(),
                    headingDelta / seconds);
            previousDistances = distances;
            previousHeadingRadians = heading;
        }
        measurement = Measurements.poseAndSpeeds(pose, velocity)
                .timing(timestampSeconds, 0.0)
                .source(this);
    }

    @Override
    public synchronized List<Measurement> measurements() {
        return measurement == null ? List.of() : List.of(measurement);
    }

    @Override
    public synchronized void reset(PoseSnapshot newPose) {
        pendingReset = Objects.requireNonNull(newPose, "pose");
        pose = newPose;
        velocity = RobotVelocity.zero();
    }

    public synchronized PoseSnapshot pose() {
        return pose;
    }

    public synchronized RobotVelocity velocity() {
        return velocity;
    }

    private static EncoderDevice distanceEncoder(SwerveModule module) {
        return module.drive.get().encoder()
                .gearRatio(GearRatio.reduction(module.model().driveReduction(), 1.0))
                .wheelDiameterMeters(module.model().wheelDiameterMeters())
                .units(EncoderUnit.METERS);
    }

    private static double wrapRadians(double radians) {
        return radians - Math.floor((radians + Math.PI) / TWO_PI) * TWO_PI;
    }
}
