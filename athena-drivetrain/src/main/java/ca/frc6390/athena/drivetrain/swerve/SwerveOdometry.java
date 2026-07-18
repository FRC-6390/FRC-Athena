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
import java.util.List;
import java.util.Objects;

/** Runtime-refreshed odometry derived from measured swerve module travel and IMU heading. */
public final class SwerveOdometry implements PoseSignal, HardwareMeasurementSignal, ResettablePoseSignal {
    private static final double TWO_PI = Math.PI * 2.0;

    private final SwerveKinematics kinematics;
    private final ImuSource imu;
    private final List<EncoderDevice> driveEncoders;
    private final double[][] velocityInverse;
    private final double[] moduleSpeedsMetersPerSecond;
    private double[] distanceReadings;
    private double[] previousDistances;
    private double[] angleReadings;
    private double[] previousAngles;
    private boolean hasPreviousReadings;
    private PoseSnapshot pose = new PoseSnapshot(0.0, 0.0, 0.0);
    private PoseSnapshot pendingReset;
    private RobotVelocity velocity = RobotVelocity.zero();
    private List<ModulePosition> modulePositionSnapshot = List.of();
    private long modulePositionVersion;
    private long snapshottedModulePositionVersion = -1L;
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
        int moduleCount = driveEncoders.size();
        distanceReadings = new double[moduleCount];
        previousDistances = new double[moduleCount];
        angleReadings = new double[moduleCount];
        previousAngles = new double[moduleCount];
        moduleSpeedsMetersPerSecond = new double[moduleCount];
        velocityInverse = velocityInverse(kinematics.modules());
    }

    @Override
    public synchronized void refresh(ActionContext context, double timestampSeconds, double dtSeconds) {
        Objects.requireNonNull(context, "context");
        if (Double.compare(lastRefreshTimestamp, timestampSeconds) == 0) {
            return;
        }
        for (int index = 0; index < driveEncoders.size(); index++) {
            EncoderDevice driveEncoder = driveEncoders.get(index);
            EncoderDevice angleEncoder = kinematics.modules().get(index).module().angle.get();
            distanceReadings[index] = RuntimeHardwareAccess.call(context, driveEncoder::position);
            angleReadings[index] = RuntimeHardwareAccess.call(context, angleEncoder::absolutePosition);
        }

        double rawHeading = Math.toRadians(imu.yawDegrees());
        if (pendingReset != null) {
            pose = pendingReset;
            headingOffsetRadians = pose.headingRadians() - rawHeading;
            pendingReset = null;
            commitModuleReadings();
            previousHeadingRadians = pose.headingRadians();
            velocity = RobotVelocity.zero();
        } else if (!hasPreviousReadings) {
            headingOffsetRadians = pose.headingRadians() - rawHeading;
            commitModuleReadings();
            previousHeadingRadians = pose.headingRadians();
        } else {
            double seconds = Double.isFinite(dtSeconds) && dtSeconds > 0.0 ? dtSeconds : 0.02;
            for (int index = 0; index < distanceReadings.length; index++) {
                moduleSpeedsMetersPerSecond[index] =
                        (distanceReadings[index] - previousDistances[index]) / seconds;
            }
            RobotVelocity wheelVelocity = measuredVelocity();
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
            commitModuleReadings();
            previousHeadingRadians = heading;
        }
        modulePositionVersion++;
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
        if (!hasPreviousReadings) {
            return List.of();
        }
        if (snapshottedModulePositionVersion != modulePositionVersion) {
            ModulePosition[] positions = new ModulePosition[previousDistances.length];
            for (int index = 0; index < positions.length; index++) {
                positions[index] = new ModulePosition(previousDistances[index], previousAngles[index]);
            }
            modulePositionSnapshot = List.of(positions);
            snapshottedModulePositionVersion = modulePositionVersion;
        }
        return modulePositionSnapshot;
    }

    private void commitModuleReadings() {
        double[] distanceSwap = previousDistances;
        previousDistances = distanceReadings;
        distanceReadings = distanceSwap;

        double[] angleSwap = previousAngles;
        previousAngles = angleReadings;
        angleReadings = angleSwap;
        hasPreviousReadings = true;
    }

    private RobotVelocity measuredVelocity() {
        if (velocityInverse == null) {
            return RobotVelocity.zero();
        }
        double rhsX = 0.0;
        double rhsY = 0.0;
        double rhsRotation = 0.0;
        for (int index = 0; index < moduleSpeedsMetersPerSecond.length; index++) {
            SwerveKinematics.Module module = kinematics.modules().get(index);
            double angleRadians = angleReadings[index] * TWO_PI;
            double wheelX = moduleSpeedsMetersPerSecond[index] * Math.cos(angleRadians);
            double wheelY = moduleSpeedsMetersPerSecond[index] * Math.sin(angleRadians);
            rhsX += wheelX;
            rhsY += wheelY;
            rhsRotation += -module.yMeters() * wheelX + module.xMeters() * wheelY;
        }
        return new RobotVelocity(
                dot(velocityInverse[0], rhsX, rhsY, rhsRotation),
                dot(velocityInverse[1], rhsX, rhsY, rhsRotation),
                dot(velocityInverse[2], rhsX, rhsY, rhsRotation));
    }

    private static double[][] velocityInverse(List<SwerveKinematics.Module> modules) {
        double sumX = 0.0;
        double sumY = 0.0;
        double sumRadiusSquared = 0.0;
        for (SwerveKinematics.Module module : modules) {
            sumX += module.xMeters();
            sumY += module.yMeters();
            sumRadiusSquared += module.xMeters() * module.xMeters()
                    + module.yMeters() * module.yMeters();
        }
        double moduleCount = modules.size();
        return inverse3x3(new double[][] {
            {moduleCount, 0.0, -sumY},
            {0.0, moduleCount, sumX},
            {-sumY, sumX, sumRadiusSquared}
        });
    }

    private static double[][] inverse3x3(double[][] matrix) {
        double a = matrix[0][0];
        double b = matrix[0][1];
        double c = matrix[0][2];
        double d = matrix[1][0];
        double e = matrix[1][1];
        double f = matrix[1][2];
        double g = matrix[2][0];
        double h = matrix[2][1];
        double i = matrix[2][2];
        double determinant = a * (e * i - f * h)
                - b * (d * i - f * g)
                + c * (d * h - e * g);
        if (Math.abs(determinant) < 1.0e-12) {
            return null;
        }
        double scale = 1.0 / determinant;
        return new double[][] {
            {(e * i - f * h) * scale, (c * h - b * i) * scale, (b * f - c * e) * scale},
            {(f * g - d * i) * scale, (a * i - c * g) * scale, (c * d - a * f) * scale},
            {(d * h - e * g) * scale, (b * g - a * h) * scale, (a * e - b * d) * scale}
        };
    }

    private static double dot(double[] row, double x, double y, double rotation) {
        return row[0] * x + row[1] * y + row[2] * rotation;
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
