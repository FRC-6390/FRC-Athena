package ca.frc6390.athena.vendor.limelight;

import ca.frc6390.athena.api.hardware.AthenaCamera;
import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.vision.spec.CameraSpec;
import ca.frc6390.athena.vision.spec.VisionFrame;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.Objects;

/**
 * Limelight camera adapter backed by Limelight NetworkTables values.
 */
public final class LimelightCameraAdapter {
    private static final String DEFAULT_TABLE_NAME = "limelight";
    private static final double[] EMPTY_POSE = new double[0];

    private final LimelightClient client;

    /**
     * Creates an adapter backed by the default NetworkTables instance and the
     * default Limelight table name.
     */
    public LimelightCameraAdapter() {
        this(NetworkTableInstance.getDefault(), DEFAULT_TABLE_NAME);
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
        return kind == AthenaCamera.LIMELIGHT || kind.key().equals("limelight:camera");
    }

    /**
     * Returns true when a camera spec is handled by this adapter.
     *
     * @param spec camera spec
     * @return true if supported
     */
    public boolean supports(CameraSpec spec) {
        return spec != null && supports(spec.kind());
    }

    /**
     * Converts a Limelight-shaped target into Athena's generic frame model.
     *
     * @param target target values
     * @return vision frame
     */
    public VisionFrame frame(LimelightTarget target) {
        return frameFromTarget(target);
    }

    /**
     * Reads the latest Limelight NetworkTables values and converts them to an
     * Athena vision frame.
     *
     * @return latest vision frame
     */
    public VisionFrame latestFrame() {
        return frame(client.latestTarget());
    }

    /**
     * Converts a Limelight-shaped target into Athena's generic frame model.
     *
     * @param target target values
     * @return vision frame
     */
    public static VisionFrame frameFromTarget(LimelightTarget target) {
        if (target == null || !target.hasTarget()) {
            return VisionFrame.noTarget();
        }
        return VisionFrame.of(target.toObservation());
    }

    interface LimelightClient {
        LimelightTarget latestTarget();
    }

    private static final class NetworkTablesLimelightClient implements LimelightClient {
        private final NetworkTable table;

        private NetworkTablesLimelightClient(NetworkTableInstance instance, String tableName) {
            Objects.requireNonNull(instance, "instance");
            table = instance.getTable(normalizeTableName(tableName));
        }

        @Override
        public LimelightTarget latestTarget() {
            boolean hasTarget = table.getEntry("tv").getDouble(0.0) >= 1.0;
            if (!hasTarget) {
                return LimelightTarget.noTarget();
            }
            double[] targetPose = table.getEntry("targetpose_cameraspace").getDoubleArray(EMPTY_POSE);
            return LimelightTarget.aprilTag(
                    (int) Math.round(table.getEntry("tid").getDouble(-1.0)),
                    table.getEntry("tx").getDouble(0.0),
                    table.getEntry("ty").getDouble(0.0),
                    distanceMeters(targetPose),
                    table.getEntry("ta").getDouble(0.0));
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
}
