package ca.frc6390.athena.core.localization;

import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicReference;

import ca.frc6390.athena.core.RobotTime;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;

final class RobotLocalizationPersistence {

    private static final String PERSISTENCE_PREFIX = "Shuffleboard/Localization/Save Data/";
    private static final String PREF_FIELD_X = PERSISTENCE_PREFIX + "FieldPoseX";
    private static final String PREF_FIELD_Y = PERSISTENCE_PREFIX + "FieldPoseY";
    private static final String PREF_FIELD_Z = PERSISTENCE_PREFIX + "FieldPoseZ";
    private static final String PREF_FIELD_ROLL_DEG = PERSISTENCE_PREFIX + "FieldPoseRollDeg";
    private static final String PREF_FIELD_PITCH_DEG = PERSISTENCE_PREFIX + "FieldPosePitchDeg";
    private static final String PREF_FIELD_YAW_DEG = PERSISTENCE_PREFIX + "FieldPoseYawDeg";
    private static final String PREF_DRIVER_YAW_DEG = PERSISTENCE_PREFIX + "DriverYawDeg";
    private static final String PREF_VALID = PERSISTENCE_PREFIX + "Valid";
    private static final double TRANSLATION_EPSILON_METERS = 0.005;
    private static final double ROTATION_EPSILON_RADIANS = Units.degreesToRadians(0.25);
    private static final double PERSISTENCE_INTERVAL_SECONDS = 1.5;
    private static final AtomicInteger PERSIST_THREAD_COUNTER = new AtomicInteger(1);

    private final RobotLocalizationConfig.PoseSpace poseSpace;
    private final ExecutorService persistWorker = Executors.newSingleThreadExecutor(persistThreadFactory());
    private final AtomicReference<PersistRequest> pendingPersistRequest = new AtomicReference<>();
    private final AtomicBoolean persistWorkerScheduled = new AtomicBoolean(false);

    private Pose2d lastPose2d;
    private Pose3d lastPose3d;
    private Rotation2d lastDriverYaw;
    private double lastPersistTimestampSeconds = Double.NEGATIVE_INFINITY;
    private double lastPersistedFieldX = Double.NaN;
    private double lastPersistedFieldY = Double.NaN;
    private double lastPersistedFieldZ = Double.NaN;
    private double lastPersistedFieldRollDeg = Double.NaN;
    private double lastPersistedFieldPitchDeg = Double.NaN;
    private double lastPersistedFieldYawDeg = Double.NaN;
    private double lastPersistedDriverYawDeg = Double.NaN;
    private boolean lastPersistedValid = false;
    private boolean hasPersistedSnapshot = false;

    RobotLocalizationPersistence(RobotLocalizationConfig.PoseSpace poseSpace) {
        this.poseSpace = poseSpace;
    }

    void updateSnapshot(Pose2d pose2d, Pose3d pose3d, Rotation2d driverYaw) {
        this.lastPose2d = pose2d;
        this.lastPose3d = pose3d;
        this.lastDriverYaw = driverYaw;
    }

    void persist(Pose2d pose2d, Pose3d pose3d, Rotation2d driverYaw, boolean force, boolean skipPersist) {
        if (pose2d == null || driverYaw == null || skipPersist) {
            updateSnapshot(pose2d, pose3d, driverYaw);
            return;
        }

        if (!isFinitePose(pose2d)) {
            return;
        }

        boolean poseChanged = hasPoseChanged(lastPose2d, pose2d);
        boolean yawChanged = hasRotationChanged(lastDriverYaw, driverYaw);
        double now = currentTimestampSeconds();

        if (!force) {
            if (!poseChanged && !yawChanged) {
                return;
            }
            if ((now - lastPersistTimestampSeconds) < PERSISTENCE_INTERVAL_SECONDS) {
                return;
            }
        }

        double fieldX = pose2d.getX();
        double fieldY = pose2d.getY();

        double fieldZ;
        double fieldRollDeg;
        double fieldPitchDeg;
        double fieldYawDeg;
        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D && pose3d != null) {
            double z = pose3d.getZ();
            Rotation3d rotation3d = pose3d.getRotation();
            fieldZ = Double.isFinite(z) ? z : 0.0;
            fieldRollDeg = Units.radiansToDegrees(rotation3d.getX());
            fieldPitchDeg = Units.radiansToDegrees(rotation3d.getY());
            fieldYawDeg = Units.radiansToDegrees(rotation3d.getZ());
        } else {
            fieldZ = 0.0;
            fieldRollDeg = 0.0;
            fieldPitchDeg = 0.0;
            fieldYawDeg = pose2d.getRotation().getDegrees();
        }
        double driverYawDeg = driverYaw.getDegrees();

        enqueuePersistRequest(new PersistRequest(
                fieldX,
                fieldY,
                fieldZ,
                fieldRollDeg,
                fieldPitchDeg,
                fieldYawDeg,
                driverYawDeg,
                true));

        lastPersistedFieldX = fieldX;
        lastPersistedFieldY = fieldY;
        lastPersistedFieldZ = fieldZ;
        lastPersistedFieldRollDeg = fieldRollDeg;
        lastPersistedFieldPitchDeg = fieldPitchDeg;
        lastPersistedFieldYawDeg = fieldYawDeg;
        lastPersistedDriverYawDeg = driverYawDeg;
        lastPersistedValid = true;
        hasPersistedSnapshot = true;

        lastPersistTimestampSeconds = now;
        updateSnapshot(pose2d, pose3d, driverYaw);
    }

    Optional<PersistentState> readPersistentState() {
        boolean hasValidStoredState =
                Preferences.containsKey(PREF_VALID) && Preferences.getBoolean(PREF_VALID, false);
        if (!hasValidStoredState) {
            return Optional.empty();
        }

        double storedX = Preferences.getDouble(PREF_FIELD_X, 0.0);
        double storedY = Preferences.getDouble(PREF_FIELD_Y, 0.0);
        double storedYawDeg = Preferences.getDouble(PREF_FIELD_YAW_DEG, 0.0);
        double storedDriverYawDeg = Preferences.getDouble(PREF_DRIVER_YAW_DEG, 0.0);

        if (!Double.isFinite(storedX) || !Double.isFinite(storedY) || !Double.isFinite(storedYawDeg)) {
            return Optional.empty();
        }

        Pose2d pose2d = new Pose2d(storedX, storedY, Rotation2d.fromDegrees(storedYawDeg));
        Rotation2d driverYaw = Rotation2d.fromDegrees(storedDriverYawDeg);
        Pose3d pose3d;

        if (poseSpace == RobotLocalizationConfig.PoseSpace.THREE_D) {
            double storedZ = Preferences.getDouble(PREF_FIELD_Z, 0.0);
            double storedRollDeg = Preferences.getDouble(PREF_FIELD_ROLL_DEG, 0.0);
            double storedPitchDeg = Preferences.getDouble(PREF_FIELD_PITCH_DEG, 0.0);
            Rotation3d rotation3d = new Rotation3d(
                    Units.degreesToRadians(storedRollDeg),
                    Units.degreesToRadians(storedPitchDeg),
                    Units.degreesToRadians(storedYawDeg));
            pose3d = new Pose3d(storedX, storedY, storedZ, rotation3d);
        } else {
            pose3d = new Pose3d(pose2d);
        }

        lastPersistTimestampSeconds = currentTimestampSeconds();
        lastPersistedFieldX = storedX;
        lastPersistedFieldY = storedY;
        lastPersistedFieldZ = pose3d.getZ();
        lastPersistedFieldRollDeg = Units.radiansToDegrees(pose3d.getRotation().getX());
        lastPersistedFieldPitchDeg = Units.radiansToDegrees(pose3d.getRotation().getY());
        lastPersistedFieldYawDeg = storedYawDeg;
        lastPersistedDriverYawDeg = storedDriverYawDeg;
        lastPersistedValid = true;
        hasPersistedSnapshot = true;
        updateSnapshot(pose2d, pose3d, driverYaw);
        return Optional.of(new PersistentState(pose2d, pose3d, driverYaw));
    }

    private void enqueuePersistRequest(PersistRequest request) {
        if (request == null) {
            return;
        }
        pendingPersistRequest.set(request);
        if (persistWorkerScheduled.compareAndSet(false, true)) {
            persistWorker.execute(this::drainPersistRequests);
        }
    }

    private void drainPersistRequests() {
        while (true) {
            PersistRequest request = pendingPersistRequest.getAndSet(null);
            if (request == null) {
                persistWorkerScheduled.set(false);
                if (pendingPersistRequest.get() != null && persistWorkerScheduled.compareAndSet(false, true)) {
                    continue;
                }
                return;
            }
            writePersistRequest(request);
        }
    }

    private static void writePersistRequest(PersistRequest request) {
        if (request == null) {
            return;
        }
        Preferences.setDouble(PREF_FIELD_X, request.fieldX());
        Preferences.setDouble(PREF_FIELD_Y, request.fieldY());
        Preferences.setDouble(PREF_FIELD_Z, request.fieldZ());
        Preferences.setDouble(PREF_FIELD_ROLL_DEG, request.fieldRollDeg());
        Preferences.setDouble(PREF_FIELD_PITCH_DEG, request.fieldPitchDeg());
        Preferences.setDouble(PREF_FIELD_YAW_DEG, request.fieldYawDeg());
        Preferences.setDouble(PREF_DRIVER_YAW_DEG, request.driverYawDeg());
        Preferences.setBoolean(PREF_VALID, request.valid());
    }

    private static ThreadFactory persistThreadFactory() {
        return runnable -> {
            Thread thread = new Thread(
                    runnable,
                    "athena-localization-persist-" + PERSIST_THREAD_COUNTER.getAndIncrement());
            thread.setDaemon(true);
            return thread;
        };
    }

    private static double currentTimestampSeconds() {
        double nowSeconds = RobotTime.nowSeconds();
        if (Double.isFinite(nowSeconds)) {
            return nowSeconds;
        }
        return edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    }

    private static boolean isFinitePose(Pose2d pose) {
        return pose != null
                && Double.isFinite(pose.getX())
                && Double.isFinite(pose.getY())
                && Double.isFinite(pose.getRotation().getRadians());
    }

    private static boolean hasPoseChanged(Pose2d previous, Pose2d current) {
        if (previous == null || current == null) {
            return true;
        }
        double dx = current.getX() - previous.getX();
        double dy = current.getY() - previous.getY();
        double translationDelta = Math.hypot(dx, dy);
        double rotationDelta = Math.abs(current.getRotation().minus(previous.getRotation()).getRadians());
        return translationDelta > TRANSLATION_EPSILON_METERS
                || rotationDelta > ROTATION_EPSILON_RADIANS;
    }

    private static boolean hasRotationChanged(Rotation2d previous, Rotation2d current) {
        if (previous == null || current == null) {
            return true;
        }
        return Math.abs(previous.minus(current).getRadians()) > ROTATION_EPSILON_RADIANS;
    }

    private record PersistRequest(
            double fieldX,
            double fieldY,
            double fieldZ,
            double fieldRollDeg,
            double fieldPitchDeg,
            double fieldYawDeg,
            double driverYawDeg,
            boolean valid) {}

    record PersistentState(Pose2d pose2d, Pose3d pose3d, Rotation2d driverYaw) {}
}
