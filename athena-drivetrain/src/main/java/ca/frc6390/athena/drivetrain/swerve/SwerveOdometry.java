package ca.frc6390.athena.drivetrain.swerve;

import ca.frc6390.athena.hardware.device.EncoderDevice;
import ca.frc6390.athena.hardware.device.GearRatio;
import ca.frc6390.athena.hardware.encoder.EncoderUnit;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.HardwareMeasurementSignal;
import ca.frc6390.athena.hardware.runtime.RuntimeHardwareAccess;
import ca.frc6390.athena.hardware.signal.ImuSource;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import ca.frc6390.athena.runtime.measurement.ResettablePoseSignal;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

/** Runtime-refreshed odometry derived from measured swerve module travel and IMU heading. */
public final class SwerveOdometry implements PoseSignal, HardwareMeasurementSignal, ResettablePoseSignal {
    private static final double TWO_PI = Math.PI * 2.0;

    private final SwerveKinematics kinematics;
    private final ImuSource imu;
    private final List<EncoderDevice> driveEncoders;
    private double[] previousDistances;
    private PoseSnapshot pose = new PoseSnapshot(0.0, 0.0, 0.0);
    private PoseSnapshot pendingReset;
    private RobotVelocity velocity = RobotVelocity.zero();
    private List<ModulePosition> modulePositions = List.of();
    private double headingOffsetRadians;
    private double previousHeadingRadians;
    private double lastRefreshTimestamp = Double.NaN;
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
        if (Double.compare(lastRefreshTimestamp, timestampSeconds) == 0) {
            return;
        }
        double[] distances = new double[driveEncoders.size()];
        List<Double> angles = new ArrayList<>(driveEncoders.size());
        for (int index = 0; index < driveEncoders.size(); index++) {
            EncoderDevice driveEncoder = driveEncoders.get(index);
            EncoderDevice angleEncoder = kinematics.modules().get(index).module().angle.get();
            distances[index] = RuntimeHardwareAccess.call(context, driveEncoder::position);
            angles.add(RuntimeHardwareAccess.call(context, angleEncoder::absolutePosition));
        }
        List<ModulePosition> positions = new ArrayList<>(distances.length);
        for (int index = 0; index < distances.length; index++) {
            positions.add(new ModulePosition(distances[index], angles.get(index)));
        }
        modulePositions = List.copyOf(positions);

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
        lastRefreshTimestamp = timestampSeconds;
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

    /** Returns the kinematic layout represented by this odometry source. */
    public SwerveKinematics kinematics() {
        return kinematics;
    }

    /** Returns the latest gyro-backed heading used by odometry. */
    public synchronized double headingRadians() {
        return pose.headingRadians();
    }

    /** Returns the latest measured module positions in kinematic order. */
    public synchronized List<ModulePosition> modulePositions() {
        return modulePositions;
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

    /** Measured wheel travel and azimuth for one positioned module. */
    public record ModulePosition(double distanceMeters, double angleRotations) {
    }
}
