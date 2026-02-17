package ca.frc6390.athena.core.localization;

import java.util.HashMap;
import java.util.Map;

import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.sensors.camera.VisionCamera.CoordinateSpace;
import ca.frc6390.athena.sensors.camera.VisionCamera.TargetObservation;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

/**
 * Manages camera visualization + tuning via raw NetworkTables topics (no Shuffleboard containers).
 *
 * <p>Config/toggles are published under {@code /Athena/NetworkTableConfig/Vision/Cameras/<key>/...} so Elastic/Glass
 * (or custom tools) can change behavior at runtime without redeploying.</p>
 */
final class RobotVisionCameraManager {

    private static final Pose2d ZERO_POSE = new Pose2d();
    private static final java.util.List<Pose2d> EMPTY_POSES = java.util.List.of();
    private static final double CAMERA_CONFIG_SYNC_PERIOD_SECONDS = 0.25;
    private static final double STD_THETA_EPSILON_RAD = Math.toRadians(0.01);
    private static final double TAG_LINE_POSE_EPSILON_METERS = 1e-6;

    private final double stdEpsilon;
    private final Field2d field;
    private final Field2d visionField;
    private final Map<String, CameraDisplayState> cameraDisplayStates = new HashMap<>();
    private RobotNetworkTables robotNetworkTables;

    RobotVisionCameraManager(double stdEpsilon, Field2d field) {
        this.stdEpsilon = stdEpsilon;
        this.field = field;
        this.visionField = new Field2d();
    }

    void attachRobotNetworkTables(RobotNetworkTables robotNetworkTables) {
        this.robotNetworkTables = robotNetworkTables;
    }

    Field2d getVisionField() {
        return visionField;
    }

    void ensureCameraEntries(RobotVision vision) {
        if (vision == null) {
            return;
        }
        Map<String, VisionCamera> cameras = vision.cameras().all();
        for (Map.Entry<String, VisionCamera> entry : cameras.entrySet()) {
            cameraDisplayStates.computeIfAbsent(entry.getKey(), key -> createCameraDisplayState(key, entry.getValue()));
        }
        cameraDisplayStates.entrySet().removeIf(stateEntry -> {
            if (!cameras.containsKey(stateEntry.getKey())) {
                CameraDisplayState state = stateEntry.getValue();
                state.cameraPosePublisher.close();
                state.estimatedPosePublisher.close();
                state.visionCameraPoseObject.setPoses(EMPTY_POSES);
                state.localizationEstimatedPoseObject.setPoses(EMPTY_POSES);
                state.visionTagLineObject.setPoses(EMPTY_POSES);
                return true;
            }
            return false;
        });
    }

    void updateCameraVisualizations(RobotVision vision, Pose2d fieldPose) {
        if (vision == null) {
            return;
        }
        ensureCameraEntries(vision);
        for (CameraDisplayState state : cameraDisplayStates.values()) {
            VisionCamera camera = vision.cameras().camera(state.key);
            if (camera == null) {
                state.visionCameraPoseObject.setPoses(EMPTY_POSES);
                state.localizationEstimatedPoseObject.setPoses(EMPTY_POSES);
                state.visionTagLineObject.setPoses(EMPTY_POSES);
                state.cameraPosePublisher.set(ZERO_POSE);
                state.estimatedPosePublisher.set(ZERO_POSE);
                continue;
            }
            applyCameraConfigUpdates(state, camera, vision);
            Pose2d cameraPose = computeCameraFieldPose(camera, fieldPose);
            updateCameraPose(state, cameraPose);
            updateEstimatedPose(state, camera);
            updateTagLines(state, camera, cameraPose, fieldPose);
        }
    }

    private CameraDisplayState createCameraDisplayState(String key, VisionCamera camera) {
        if (key == null || key.isBlank() || camera == null) {
            return null;
        }

        RobotNetworkTables nt = robotNetworkTables;
        if (nt == null) {
            return null;
        }
        RobotNetworkTables.Node cfg = nt.root()
                .child("NetworkTableConfig")
                .child("Vision")
                .child("Cameras")
                .child(key);

        NetworkTableEntry showCameraPoseEntry = initIfAbsent(cfg.entry("ShowCameraPose"), true);
        NetworkTableEntry showTagLinesEntry = initIfAbsent(cfg.entry("ShowTagLines"), true);
        NetworkTableEntry showEstimatedPoseEntry = initIfAbsent(cfg.entry("ShowEstimatedPose"), false);

        boolean defaultUseForLocalization = camera.isUseForLocalization();
        NetworkTableEntry useForLocalizationEntry = initIfAbsent(cfg.entry("UseForLocalization"), defaultUseForLocalization);

        double trustDistance = camera.getTrustDistance();
        NetworkTableEntry trustDistanceEntry = initIfAbsent(cfg.entry("TrustDistanceMeters"), trustDistance);

        double singleStdX = camera.getSingleStdDev().get(0, 0);
        double singleStdY = camera.getSingleStdDev().get(1, 0);
        double singleStdThetaDeg = Math.toDegrees(camera.getSingleStdDev().get(2, 0));
        NetworkTableEntry singleStdXEntry = initIfAbsent(cfg.entry("SingleStdDevX_M"), singleStdX);
        NetworkTableEntry singleStdYEntry = initIfAbsent(cfg.entry("SingleStdDevY_M"), singleStdY);
        NetworkTableEntry singleStdThetaDegEntry = initIfAbsent(cfg.entry("SingleStdDevTheta_Deg"), singleStdThetaDeg);

        double multiStdX = camera.getMultiStdDev().get(0, 0);
        double multiStdY = camera.getMultiStdDev().get(1, 0);
        double multiStdThetaDeg = Math.toDegrees(camera.getMultiStdDev().get(2, 0));
        NetworkTableEntry multiStdXEntry = initIfAbsent(cfg.entry("MultiStdDevX_M"), multiStdX);
        NetworkTableEntry multiStdYEntry = initIfAbsent(cfg.entry("MultiStdDevY_M"), multiStdY);
        NetworkTableEntry multiStdThetaDegEntry = initIfAbsent(cfg.entry("MultiStdDevTheta_Deg"), multiStdThetaDeg);

        FieldObject2d cameraPoseObject = visionField.getObject("CameraPose/" + key);
        FieldObject2d estimatedPoseObject = field.getObject("CameraEstimate/" + key);
        FieldObject2d tagLineObject = visionField.getObject("CameraTagLine/" + key);
        cameraPoseObject.setPoses(EMPTY_POSES);
        estimatedPoseObject.setPoses(EMPTY_POSES);
        tagLineObject.setPoses(EMPTY_POSES);

        StructPublisher<Pose2d> cameraPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("/Athena/Vision/Cameras/" + key + "/CameraPose", Pose2d.struct)
                .publish();
        cameraPosePublisher.set(ZERO_POSE);

        StructPublisher<Pose2d> estimatedPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("/Athena/Localization/Cameras/" + key + "/EstimatedPose", Pose2d.struct)
                .publish();
        estimatedPosePublisher.set(ZERO_POSE);

        CameraDisplayState state = new CameraDisplayState(
                key,
                showCameraPoseEntry,
                showTagLinesEntry,
                showEstimatedPoseEntry,
                useForLocalizationEntry,
                trustDistanceEntry,
                singleStdXEntry,
                singleStdYEntry,
                singleStdThetaDegEntry,
                multiStdXEntry,
                multiStdYEntry,
                multiStdThetaDegEntry,
                cameraPoseObject,
                estimatedPoseObject,
                tagLineObject,
                cameraPosePublisher,
                estimatedPosePublisher);
        state.useForLocalizationMirrored = true;
        state.lastUseForLocalizationValue = defaultUseForLocalization;
        state.trustDistanceMirrored = true;
        state.lastTrustDistanceValue = trustDistance;
        state.singleStdXMirrored = true;
        state.lastSingleStdXValue = singleStdX;
        state.singleStdYMirrored = true;
        state.lastSingleStdYValue = singleStdY;
        state.singleStdThetaMirrored = true;
        state.lastSingleStdThetaDegValue = singleStdThetaDeg;
        state.multiStdXMirrored = true;
        state.lastMultiStdXValue = multiStdX;
        state.multiStdYMirrored = true;
        state.lastMultiStdYValue = multiStdY;
        state.multiStdThetaMirrored = true;
        state.lastMultiStdThetaDegValue = multiStdThetaDeg;
        return state;
    }

    private void applyCameraConfigUpdates(CameraDisplayState state, VisionCamera camera, RobotVision vision) {
        if (state == null || camera == null) {
            return;
        }
        double nowSeconds = Timer.getFPGATimestamp();
        if (Double.isFinite(nowSeconds)
                && Double.isFinite(state.lastConfigSyncSeconds)
                && nowSeconds - state.lastConfigSyncSeconds < CAMERA_CONFIG_SYNC_PERIOD_SECONDS) {
            return;
        }
        state.lastConfigSyncSeconds = nowSeconds;

        boolean currentUse = camera.isUseForLocalization();
        boolean desiredUse = state.useForLocalizationEntry != null
                ? state.useForLocalizationEntry.getBoolean(currentUse)
                : currentUse;
        if (desiredUse != currentUse) {
            if (vision != null) {
                vision.camera(state.key, runtime -> runtime.useForLocalization(desiredUse));
            } else {
                camera.config().useForLocalization(desiredUse);
            }
        }
        mirrorUseForLocalization(state, camera.isUseForLocalization());

        double currentTrust = camera.getTrustDistance();
        double desiredTrust = state.trustDistanceEntry != null
                ? state.trustDistanceEntry.getDouble(currentTrust)
                : currentTrust;
        if (Math.abs(desiredTrust - currentTrust) > stdEpsilon) {
            camera.config().trustDistance(desiredTrust);
        }
        mirrorTrustDistance(state, camera.getTrustDistance());

        Matrix<N3, N1> singleStd = camera.getSingleStdDev();
        double desiredSingleX = sanitizeStdDev(state.singleStdXEntry.getDouble(singleStd.get(0, 0)));
        double desiredSingleY = sanitizeStdDev(state.singleStdYEntry.getDouble(singleStd.get(1, 0)));
        double desiredSingleThetaDeg =
                state.singleStdThetaDegEntry.getDouble(Math.toDegrees(singleStd.get(2, 0)));
        double desiredSingleThetaRad = Math.toRadians(desiredSingleThetaDeg);
        if (Math.abs(desiredSingleX - singleStd.get(0, 0)) > stdEpsilon
                || Math.abs(desiredSingleY - singleStd.get(1, 0)) > stdEpsilon
                || Math.abs(desiredSingleThetaRad - singleStd.get(2, 0)) > STD_THETA_EPSILON_RAD) {
            Matrix<N3, N1> updatedSingle = VecBuilder.fill(desiredSingleX, desiredSingleY, desiredSingleThetaRad);
            camera.config().singleStdDevs(updatedSingle);
        }
        Matrix<N3, N1> appliedSingle = camera.getSingleStdDev();
        mirrorSingleStdDevs(state, appliedSingle);

        Matrix<N3, N1> multiStd = camera.getMultiStdDev();
        double desiredMultiX = sanitizeStdDev(state.multiStdXEntry.getDouble(multiStd.get(0, 0)));
        double desiredMultiY = sanitizeStdDev(state.multiStdYEntry.getDouble(multiStd.get(1, 0)));
        double desiredMultiThetaDeg =
                state.multiStdThetaDegEntry.getDouble(Math.toDegrees(multiStd.get(2, 0)));
        double desiredMultiThetaRad = Math.toRadians(desiredMultiThetaDeg);
        if (Math.abs(desiredMultiX - multiStd.get(0, 0)) > stdEpsilon
                || Math.abs(desiredMultiY - multiStd.get(1, 0)) > stdEpsilon
                || Math.abs(desiredMultiThetaRad - multiStd.get(2, 0)) > STD_THETA_EPSILON_RAD) {
            Matrix<N3, N1> updatedMulti = VecBuilder.fill(desiredMultiX, desiredMultiY, desiredMultiThetaRad);
            camera.config().multiStdDevs(updatedMulti);
        }
        Matrix<N3, N1> appliedMulti = camera.getMultiStdDev();
        mirrorMultiStdDevs(state, appliedMulti);
    }

    private void mirrorUseForLocalization(CameraDisplayState state, boolean value) {
        if (state.useForLocalizationEntry == null) {
            return;
        }
        if (!state.useForLocalizationMirrored || state.lastUseForLocalizationValue != value) {
            state.useForLocalizationEntry.setBoolean(value);
            state.lastUseForLocalizationValue = value;
            state.useForLocalizationMirrored = true;
        }
    }

    private void mirrorTrustDistance(CameraDisplayState state, double value) {
        if (state.trustDistanceEntry == null) {
            return;
        }
        if (!state.trustDistanceMirrored || Math.abs(state.lastTrustDistanceValue - value) > stdEpsilon) {
            state.trustDistanceEntry.setDouble(value);
            state.lastTrustDistanceValue = value;
            state.trustDistanceMirrored = true;
        }
    }

    private void mirrorSingleStdDevs(CameraDisplayState state, Matrix<N3, N1> singleStd) {
        if (singleStd == null) {
            return;
        }
        double singleX = sanitizeStdDev(singleStd.get(0, 0));
        if (state.singleStdXEntry != null
                && (!state.singleStdXMirrored || Math.abs(state.lastSingleStdXValue - singleX) > stdEpsilon)) {
            state.singleStdXEntry.setDouble(singleX);
            state.lastSingleStdXValue = singleX;
            state.singleStdXMirrored = true;
        }
        double singleY = sanitizeStdDev(singleStd.get(1, 0));
        if (state.singleStdYEntry != null
                && (!state.singleStdYMirrored || Math.abs(state.lastSingleStdYValue - singleY) > stdEpsilon)) {
            state.singleStdYEntry.setDouble(singleY);
            state.lastSingleStdYValue = singleY;
            state.singleStdYMirrored = true;
        }
        double singleThetaDeg = Math.toDegrees(singleStd.get(2, 0));
        if (state.singleStdThetaDegEntry != null
                && (!state.singleStdThetaMirrored || Math.abs(state.lastSingleStdThetaDegValue - singleThetaDeg) > 0.01)) {
            state.singleStdThetaDegEntry.setDouble(singleThetaDeg);
            state.lastSingleStdThetaDegValue = singleThetaDeg;
            state.singleStdThetaMirrored = true;
        }
    }

    private void mirrorMultiStdDevs(CameraDisplayState state, Matrix<N3, N1> multiStd) {
        if (multiStd == null) {
            return;
        }
        double multiX = sanitizeStdDev(multiStd.get(0, 0));
        if (state.multiStdXEntry != null
                && (!state.multiStdXMirrored || Math.abs(state.lastMultiStdXValue - multiX) > stdEpsilon)) {
            state.multiStdXEntry.setDouble(multiX);
            state.lastMultiStdXValue = multiX;
            state.multiStdXMirrored = true;
        }
        double multiY = sanitizeStdDev(multiStd.get(1, 0));
        if (state.multiStdYEntry != null
                && (!state.multiStdYMirrored || Math.abs(state.lastMultiStdYValue - multiY) > stdEpsilon)) {
            state.multiStdYEntry.setDouble(multiY);
            state.lastMultiStdYValue = multiY;
            state.multiStdYMirrored = true;
        }
        double multiThetaDeg = Math.toDegrees(multiStd.get(2, 0));
        if (state.multiStdThetaDegEntry != null
                && (!state.multiStdThetaMirrored || Math.abs(state.lastMultiStdThetaDegValue - multiThetaDeg) > 0.01)) {
            state.multiStdThetaDegEntry.setDouble(multiThetaDeg);
            state.lastMultiStdThetaDegValue = multiThetaDeg;
            state.multiStdThetaMirrored = true;
        }
    }

    private double sanitizeStdDev(double value) {
        double sanitized = Math.abs(value);
        if (!Double.isFinite(sanitized) || sanitized < stdEpsilon) {
            return stdEpsilon;
        }
        return sanitized;
    }

    private Pose2d computeCameraFieldPose(VisionCamera camera, Pose2d fieldPose) {
        if (fieldPose == null) {
            return null;
        }
        Transform3d cameraToRobot = camera.getConfig().getCameraToRobotTransform();
        Transform3d robotToCamera = cameraToRobot.inverse();
        Translation2d offset = new Translation2d(robotToCamera.getX(), robotToCamera.getY())
                .rotateBy(fieldPose.getRotation());
        Rotation2d cameraRotation = fieldPose
                .getRotation()
                .plus(Rotation2d.fromRadians(cameraToRobot.getRotation().getZ()));
        return new Pose2d(fieldPose.getTranslation().plus(offset), cameraRotation);
    }

    private void updateCameraPose(CameraDisplayState state, Pose2d cameraPose) {
        boolean show = state.showCameraPoseEntry.getBoolean(true);
        if (show && cameraPose != null) {
            state.visionCameraPoseObject.setPose(cameraPose);
        } else {
            state.visionCameraPoseObject.setPoses(EMPTY_POSES);
        }
        state.cameraPosePublisher.set(cameraPose != null ? cameraPose : ZERO_POSE);
    }

    private void updateEstimatedPose(CameraDisplayState state, VisionCamera camera) {
        boolean show = state.showEstimatedPoseEntry.getBoolean(false);
        Pose2d estimatePose =
                camera.hasValidTarget() ? camera.getLocalizationPose() : null;
        if (show && estimatePose != null) {
            state.localizationEstimatedPoseObject.setPose(estimatePose);
        } else {
            state.localizationEstimatedPoseObject.setPoses(EMPTY_POSES);
        }
        state.estimatedPosePublisher.set(estimatePose != null ? estimatePose : ZERO_POSE);
    }

    private void updateTagLines(CameraDisplayState state, VisionCamera camera, Pose2d cameraPose, Pose2d fieldPose) {
        boolean show = state.showTagLinesEntry.getBoolean(true);
        if (!show || cameraPose == null) {
            state.visionTagLineObject.setPoses(EMPTY_POSES);
            return;
        }
        TargetObservation observation =
                camera.getLatestObservation(CoordinateSpace.FIELD, fieldPose, null).orElse(null);
        if (observation == null || !observation.hasTranslation()) {
            state.visionTagLineObject.setPoses(EMPTY_POSES);
            return;
        }
        Translation2d targetFieldTranslation = observation.translation();
        Pose2d tagPose = resolveTagLinePose(state, targetFieldTranslation);
        state.visionTagLineObject.setPoses(cameraPose, tagPose);
    }

    private Pose2d resolveTagLinePose(CameraDisplayState state, Translation2d translation) {
        if (state == null || translation == null) {
            return ZERO_POSE;
        }
        double x = translation.getX();
        double y = translation.getY();
        Pose2d cached = state.cachedTagLinePose;
        if (cached != null
                && Math.abs(x - state.cachedTagLineX) <= TAG_LINE_POSE_EPSILON_METERS
                && Math.abs(y - state.cachedTagLineY) <= TAG_LINE_POSE_EPSILON_METERS) {
            return cached;
        }
        Pose2d updated = new Pose2d(x, y, Rotation2d.kZero);
        state.cachedTagLinePose = updated;
        state.cachedTagLineX = x;
        state.cachedTagLineY = y;
        return updated;
    }

    private static NetworkTableEntry initIfAbsent(NetworkTableEntry entry, boolean defaultValue) {
        if (entry != null && entry.getType() == NetworkTableType.kUnassigned) {
            entry.setBoolean(defaultValue);
        }
        return entry;
    }

    private static NetworkTableEntry initIfAbsent(NetworkTableEntry entry, double defaultValue) {
        if (entry != null && entry.getType() == NetworkTableType.kUnassigned) {
            entry.setDouble(defaultValue);
        }
        return entry;
    }

    private static final class CameraDisplayState {
        final String key;
        final NetworkTableEntry showCameraPoseEntry;
        final NetworkTableEntry showTagLinesEntry;
        final NetworkTableEntry showEstimatedPoseEntry;
        final NetworkTableEntry useForLocalizationEntry;
        final NetworkTableEntry trustDistanceEntry;
        final NetworkTableEntry singleStdXEntry;
        final NetworkTableEntry singleStdYEntry;
        final NetworkTableEntry singleStdThetaDegEntry;
        final NetworkTableEntry multiStdXEntry;
        final NetworkTableEntry multiStdYEntry;
        final NetworkTableEntry multiStdThetaDegEntry;
        final FieldObject2d visionCameraPoseObject;
        final FieldObject2d localizationEstimatedPoseObject;
        final FieldObject2d visionTagLineObject;
        final StructPublisher<Pose2d> cameraPosePublisher;
        final StructPublisher<Pose2d> estimatedPosePublisher;
        double lastConfigSyncSeconds = Double.NEGATIVE_INFINITY;
        boolean useForLocalizationMirrored;
        boolean lastUseForLocalizationValue;
        boolean trustDistanceMirrored;
        double lastTrustDistanceValue = Double.NaN;
        boolean singleStdXMirrored;
        double lastSingleStdXValue = Double.NaN;
        boolean singleStdYMirrored;
        double lastSingleStdYValue = Double.NaN;
        boolean singleStdThetaMirrored;
        double lastSingleStdThetaDegValue = Double.NaN;
        boolean multiStdXMirrored;
        double lastMultiStdXValue = Double.NaN;
        boolean multiStdYMirrored;
        double lastMultiStdYValue = Double.NaN;
        boolean multiStdThetaMirrored;
        double lastMultiStdThetaDegValue = Double.NaN;
        Pose2d cachedTagLinePose;
        double cachedTagLineX = Double.NaN;
        double cachedTagLineY = Double.NaN;

        CameraDisplayState(
                String key,
                NetworkTableEntry showCameraPoseEntry,
                NetworkTableEntry showTagLinesEntry,
                NetworkTableEntry showEstimatedPoseEntry,
                NetworkTableEntry useForLocalizationEntry,
                NetworkTableEntry trustDistanceEntry,
                NetworkTableEntry singleStdXEntry,
                NetworkTableEntry singleStdYEntry,
                NetworkTableEntry singleStdThetaDegEntry,
                NetworkTableEntry multiStdXEntry,
                NetworkTableEntry multiStdYEntry,
                NetworkTableEntry multiStdThetaDegEntry,
                FieldObject2d visionCameraPoseObject,
                FieldObject2d localizationEstimatedPoseObject,
                FieldObject2d visionTagLineObject,
                StructPublisher<Pose2d> cameraPosePublisher,
                StructPublisher<Pose2d> estimatedPosePublisher) {
            this.key = key;
            this.showCameraPoseEntry = showCameraPoseEntry;
            this.showTagLinesEntry = showTagLinesEntry;
            this.showEstimatedPoseEntry = showEstimatedPoseEntry;
            this.useForLocalizationEntry = useForLocalizationEntry;
            this.trustDistanceEntry = trustDistanceEntry;
            this.singleStdXEntry = singleStdXEntry;
            this.singleStdYEntry = singleStdYEntry;
            this.singleStdThetaDegEntry = singleStdThetaDegEntry;
            this.multiStdXEntry = multiStdXEntry;
            this.multiStdYEntry = multiStdYEntry;
            this.multiStdThetaDegEntry = multiStdThetaDegEntry;
            this.visionCameraPoseObject = visionCameraPoseObject;
            this.localizationEstimatedPoseObject = localizationEstimatedPoseObject;
            this.visionTagLineObject = visionTagLineObject;
            this.cameraPosePublisher = cameraPosePublisher;
            this.estimatedPosePublisher = estimatedPosePublisher;
        }
    }
}
