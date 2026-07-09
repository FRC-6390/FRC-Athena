package ca.frc6390.athena.vendor.limelight;

import java.util.List;
import java.util.Objects;

import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.api.hardware.CameraKinds;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.measurement.TargetMeasurementSample;
import ca.frc6390.athena.vision.device.CameraDevice;
import ca.frc6390.athena.vision.device.LimelightDevice;
import ca.frc6390.athena.vision.signal.PoseSignal;
import ca.frc6390.athena.vision.signal.TargetSignal;
import ca.frc6390.athena.vision.runtime.CameraAdapter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Limelight NetworkTables adapter for Athena camera devices and signals.
 */
public final class LimelightCameraAdapter implements CameraAdapter {
    private static final String DEFAULT_TABLE_NAME = "limelight";
    private static final double[] EMPTY_POSE = new double[0];

    private final LimelightClient client;

    /**
     * Creates an adapter backed by the default NetworkTables instance and table.
     */
    public LimelightCameraAdapter() {
        this.client = null;
    }

    /**
     * Creates an adapter backed by the default NetworkTables instance.
     *
     * @param tableName Limelight NetworkTables table name
     */
    public LimelightCameraAdapter(String tableName) {
        this(NetworkTableInstance.getDefault(), tableName);
    }

    /**
     * Creates an adapter backed by a real WPILib NetworkTables instance.
     *
     * @param instance NetworkTables instance
     * @param tableName Limelight NetworkTables table name
     */
    public LimelightCameraAdapter(NetworkTableInstance instance, String tableName) {
        this(new NetworkTablesLimelightClient(instance, tableName));
    }

    LimelightCameraAdapter(LimelightClient client) {
        this.client = Objects.requireNonNull(client, "client");
    }

    /**
     * Returns true when a camera kind is handled by this adapter.
     *
     * @param kind camera kind
     * @return true if supported
     */
    public boolean supports(CameraKind kind) {
        return kind == CameraKinds.LIMELIGHT || kind != null && kind.key().equals("limelight:camera");
    }

    /**
     * Returns true when a camera device is handled by this adapter.
     *
     * @param device camera device
     * @return true if supported
     */
    public boolean supports(CameraDevice device) {
        return device instanceof LimelightDevice || device != null && supports(device.kind());
    }

    @Override
    public CameraDevice bind(CameraDevice camera) {
        if (camera instanceof LimelightDevice limelight) {
            return bind(limelight);
        }
        throw new IllegalArgumentException("Limelight adapter cannot bind camera " + camera.name());
    }

    /**
     * Binds this adapter's signals to a Limelight device declaration.
     *
     * @param device Limelight declaration
     * @return updated device declaration
     */
    public LimelightDevice bind(LimelightDevice device) {
        Objects.requireNonNull(device, "device");
        LimelightFrameSignal signal = new LimelightFrameSignal(device, client());
        return device
                .bindPose(signal::poseMeasurements)
                .bindTargets(signal::targetMeasurements);
    }

    /**
     * Creates a pose signal for a Limelight device.
     *
     * @param device Limelight declaration
     * @return pose signal
     */
    public PoseSignal poseSignal(LimelightDevice device) {
        return new LimelightPoseNetworkTablesSignal(device, client());
    }

    /**
     * Creates a target signal for a Limelight device.
     *
     * @param device Limelight declaration
     * @return target signal
     */
    public TargetSignal targetSignal(LimelightDevice device) {
        return new LimelightTargetNetworkTablesSignal(device, client());
    }

    private LimelightClient client() {
        return client == null ? new NetworkTablesLimelightClient(NetworkTableInstance.getDefault(), DEFAULT_TABLE_NAME) : client;
    }

    interface LimelightClient {
        LimelightFrame latestFrame();
    }

    record LimelightFrame(LimelightTarget target, double[] botPoseBlue) {
        LimelightFrame {
            target = target == null ? LimelightTarget.noTarget() : target;
            botPoseBlue = botPoseBlue == null ? EMPTY_POSE : botPoseBlue.clone();
        }
    }

    private static final class LimelightFrameSignal {
        private final LimelightDevice camera;
        private final LimelightClient client;
        private LimelightFrame latest = new LimelightFrame(LimelightTarget.noTarget(), EMPTY_POSE);
        private boolean poseFramePendingTarget;

        private LimelightFrameSignal(LimelightDevice camera, LimelightClient client) {
            this.camera = Objects.requireNonNull(camera, "camera");
            this.client = Objects.requireNonNull(client, "client");
        }

        private List<Measurement> poseMeasurements() {
            latest = client.latestFrame();
            poseFramePendingTarget = true;
            return measurementsFromPose(camera, latest);
        }

        private List<Measurement> targetMeasurements() {
            if (!poseFramePendingTarget) {
                latest = client.latestFrame();
            }
            poseFramePendingTarget = false;
            return measurementsFromTarget(camera, latest);
        }
    }

    private static final class LimelightPoseNetworkTablesSignal implements PoseSignal {
        private final LimelightDevice camera;
        private final LimelightClient client;

        private LimelightPoseNetworkTablesSignal(LimelightDevice camera, LimelightClient client) {
            this.camera = Objects.requireNonNull(camera, "camera");
            this.client = Objects.requireNonNull(client, "client");
        }

        @Override
        public LimelightDevice camera() {
            return camera;
        }

        @Override
        public List<Measurement> measurements() {
            return measurementsFromPose(camera, client.latestFrame());
        }
    }

    private static final class LimelightTargetNetworkTablesSignal implements TargetSignal {
        private final LimelightDevice camera;
        private final LimelightClient client;

        private LimelightTargetNetworkTablesSignal(LimelightDevice camera, LimelightClient client) {
            this.camera = Objects.requireNonNull(camera, "camera");
            this.client = Objects.requireNonNull(client, "client");
        }

        @Override
        public LimelightDevice camera() {
            return camera;
        }

        @Override
        public List<Measurement> measurements() {
            return measurementsFromTarget(camera, client.latestFrame());
        }
    }

    private record LimelightPoseMeasurement(
            PoseSnapshot pose,
            RobotVelocity speeds,
            double timestampSeconds,
            double latencySeconds,
            double ambiguity,
            int targetCount,
            MeasurementStdDevs stdDevs,
            Object source) implements PoseMeasurementSample {
    }

    private record LimelightTargetMeasurement(
            int targetId,
            double yawDegrees,
            double pitchDegrees,
            double distanceMeters,
            double confidence,
            double timestampSeconds,
            double latencySeconds,
            Object source) implements TargetMeasurementSample {
    }

    private static final class NetworkTablesLimelightClient implements LimelightClient {
        private final NetworkTable table;

        private NetworkTablesLimelightClient(NetworkTableInstance instance, String tableName) {
            Objects.requireNonNull(instance, "instance");
            table = instance.getTable(normalizeTableName(tableName));
        }

        @Override
        public LimelightFrame latestFrame() {
            boolean hasTarget = table.getEntry("tv").getDouble(0.0) >= 1.0;
            double[] targetPose = table.getEntry("targetpose_cameraspace").getDoubleArray(EMPTY_POSE);
            LimelightTarget target = hasTarget
                    ? LimelightTarget.aprilTag(
                            (int) Math.round(table.getEntry("tid").getDouble(-1.0)),
                            table.getEntry("tx").getDouble(0.0),
                            table.getEntry("ty").getDouble(0.0),
                            distanceMeters(targetPose),
                            table.getEntry("ta").getDouble(0.0))
                    : LimelightTarget.noTarget();
            return new LimelightFrame(target, table.getEntry("botpose_wpiblue").getDoubleArray(EMPTY_POSE));
        }

        private static String normalizeTableName(String tableName) {
            return tableName == null || tableName.isBlank() ? DEFAULT_TABLE_NAME : tableName.trim();
        }

        private static double distanceMeters(double[] targetPose) {
            if (targetPose == null || targetPose.length < 3) {
                return 0.0;
            }
            return Math.hypot(Math.hypot(targetPose[0], targetPose[1]), targetPose[2]);
        }
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    static List<Measurement> measurementsFromPose(Object source, LimelightFrame frame) {
        double[] pose = frame.botPoseBlue();
        if (pose.length < 6) {
            return List.of();
        }
        double latencySeconds = pose.length > 6 ? Math.max(0.0, finiteOrZero(pose[6]) / 1000.0) : 0.0;
        int targetCount = frame.target().hasTarget() ? 1 : 0;
        return List.of(new LimelightPoseMeasurement(
                new PoseSnapshot(finiteOrZero(pose[0]), finiteOrZero(pose[1]), Math.toRadians(finiteOrZero(pose[5]))),
                RobotVelocity.zero(),
                0.0,
                latencySeconds,
                0.0,
                targetCount,
                MeasurementStdDevs.of(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY),
                source));
    }

    static List<Measurement> measurementsFromTarget(Object source, LimelightFrame frame) {
        LimelightTarget target = frame.target();
        return target.hasTarget()
                ? List.of(new LimelightTargetMeasurement(
                        target.tagId(),
                        target.txDegrees(),
                        target.tyDegrees(),
                        target.distanceMeters(),
                        target.targetArea(),
                        0.0,
                        0.0,
                        source))
                : List.of();
    }
}
