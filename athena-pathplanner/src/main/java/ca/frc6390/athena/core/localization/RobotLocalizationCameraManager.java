package ca.frc6390.athena.core.localization;

import java.util.HashMap;
import java.util.Map;

import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.sensors.camera.LocalizationCamera;
import ca.frc6390.athena.sensors.camera.LocalizationCamera.TargetObservation;
import ca.frc6390.athena.sensors.camera.LocalizationCamera.CoordinateSpace;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.SimpleWidget;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

final class RobotLocalizationCameraManager {

    private static final Pose2d ZERO_POSE = new Pose2d();
    private static final Pose2d[] EMPTY_POSES = new Pose2d[0];

    private final double stdEpsilon;
    private final Field2d field;
    private final Field2d visionField;
    private final Map<String, CameraDisplayState> cameraDisplayStates = new HashMap<>();

    private ShuffleboardTab localizationTab;
    private ShuffleboardTab visionTab;

    RobotLocalizationCameraManager(double stdEpsilon, Field2d field) {
        this.stdEpsilon = stdEpsilon;
        this.field = field;
        this.visionField = new Field2d();
    }

    Field2d getVisionField() {
        return visionField;
    }

    void setLocalizationShuffleboardTab(ShuffleboardTab tab) {
        this.localizationTab = tab;
    }

    void setVisionShuffleboardTab(ShuffleboardTab tab) {
        this.visionTab = tab;
    }

    void ensureCameraShuffleboardEntries(RobotVision vision) {
        if (vision == null || localizationTab == null || visionTab == null) {
            return;
        }
        Map<String, LocalizationCamera> cameras = vision.getCameras();
        for (Map.Entry<String, LocalizationCamera> entry : cameras.entrySet()) {
            cameraDisplayStates.computeIfAbsent(entry.getKey(), key -> createCameraDisplayState(key, entry.getValue()));
        }
        cameraDisplayStates.entrySet().removeIf(stateEntry -> {
            if (!cameras.containsKey(stateEntry.getKey())) {
                stateEntry.getValue().cameraPosePublisher.close();
                stateEntry.getValue().estimatedPosePublisher.close();
                stateEntry.getValue().visionCameraPoseObject.setPoses(EMPTY_POSES);
                stateEntry.getValue().localizationEstimatedPoseObject.setPoses(EMPTY_POSES);
                stateEntry.getValue().visionTagLineObject.setPoses(EMPTY_POSES);
                return true;
            }
            return false;
        });
    }

    void updateCameraVisualizations(RobotVision vision, Pose2d fieldPose) {
        if (vision == null) {
            return;
        }
        ensureCameraShuffleboardEntries(vision);
        for (CameraDisplayState state : cameraDisplayStates.values()) {
            LocalizationCamera camera = vision.getCamera(state.key);
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

    private CameraDisplayState createCameraDisplayState(String key, LocalizationCamera camera) {
        if (visionTab == null || localizationTab == null) {
            return null;
        }
        ShuffleboardLayout cameraLayout = visionTab.getLayout("Camera/" + key, BuiltInLayouts.kGrid);

        ShuffleboardLayout toggleLayout =
                cameraLayout.getLayout("Display Toggles", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0);

        SimpleWidget showCameraWidget =
                toggleLayout.add("Show Camera Pose", true).withWidget(BuiltInWidgets.kToggleSwitch);
        GenericEntry showCameraPoseEntry = showCameraWidget.getEntry();

        SimpleWidget showTagWidget =
                toggleLayout.add("Show Tag Lines", true).withWidget(BuiltInWidgets.kToggleSwitch);
        GenericEntry showTagLinesEntry = showTagWidget.getEntry();

        boolean defaultUseForLocalization = camera.isUseForLocalization();
        SimpleWidget useForLocalizationWidget =
                toggleLayout.add("Use For Localization", defaultUseForLocalization)
                        .withWidget(BuiltInWidgets.kToggleSwitch);
        GenericEntry useForLocalizationEntry = useForLocalizationWidget.getEntry();

        ShuffleboardLayout tuningLayout =
                cameraLayout.getLayout("Tuning", BuiltInLayouts.kGrid)
                        .withSize(4, 4)
                        .withPosition(2, 0);

        double trustDistance = camera.getConfig().getTrustDistance();
        GenericEntry trustDistanceEntry =
                tuningLayout.add("Trust Distance (m)", trustDistance)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", 0.0, "max", 10.0, "block increment", 0.05))
                        .getEntry();

        double singleStdX = camera.getSingleStdDev().get(0, 0);
        double singleStdY = camera.getSingleStdDev().get(1, 0);
        double singleStdThetaDeg = Math.toDegrees(camera.getSingleStdDev().get(2, 0));
        ShuffleboardLayout singleStdLayout =
                tuningLayout.getLayout("Single Tag Std", BuiltInLayouts.kList);
        GenericEntry singleStdXEntry =
                singleStdLayout.add("X (m)", singleStdX)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry();
        GenericEntry singleStdYEntry =
                singleStdLayout.add("Y (m)", singleStdY)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry();
        GenericEntry singleStdThetaDegEntry =
                singleStdLayout.add("Theta (deg)", singleStdThetaDeg)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry();

        double multiStdX = camera.getMultiStdDev().get(0, 0);
        double multiStdY = camera.getMultiStdDev().get(1, 0);
        double multiStdThetaDeg = Math.toDegrees(camera.getMultiStdDev().get(2, 0));
        ShuffleboardLayout multiStdLayout =
                tuningLayout.getLayout("Multi Tag Std", BuiltInLayouts.kList);
        GenericEntry multiStdXEntry =
                multiStdLayout.add("X (m)", multiStdX)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry();
        GenericEntry multiStdYEntry =
                multiStdLayout.add("Y (m)", multiStdY)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry();
        GenericEntry multiStdThetaDegEntry =
                multiStdLayout.add("Theta (deg)", multiStdThetaDeg)
                        .withWidget(BuiltInWidgets.kTextView)
                        .getEntry();

        ShuffleboardLayout estimatesLayout =
                localizationTab.getLayout("Camera Estimates", BuiltInLayouts.kList);
        SimpleWidget showEstimateWidget =
                estimatesLayout.add("Show " + key + " Estimated Pose", false)
                        .withWidget(BuiltInWidgets.kToggleSwitch);
        GenericEntry showEstimatedPoseEntry = showEstimateWidget.getEntry();

        FieldObject2d cameraPoseObject = visionField.getObject("CameraPose/" + key);
        FieldObject2d estimatedPoseObject = field.getObject("CameraEstimate/" + key);
        FieldObject2d tagLineObject = visionField.getObject("CameraTagLine/" + key);
        cameraPoseObject.setPoses(EMPTY_POSES);
        estimatedPoseObject.setPoses(EMPTY_POSES);
        tagLineObject.setPoses(EMPTY_POSES);

        String visionBaseTopic = "/Shuffleboard/" + visionTab.getTitle() + "/Camera/" + key;
        StructPublisher<Pose2d> cameraPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(visionBaseTopic + "/CameraPose", Pose2d.struct)
                .publish();
        cameraPosePublisher.set(ZERO_POSE);

        String localizationBaseTopic =
                "/Shuffleboard/" + localizationTab.getTitle() + "/CameraEstimates/" + key;
        StructPublisher<Pose2d> estimatedPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(localizationBaseTopic + "/EstimatedPose", Pose2d.struct)
                .publish();
        estimatedPosePublisher.set(ZERO_POSE);

        return new CameraDisplayState(
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
    }

    private void applyCameraConfigUpdates(CameraDisplayState state, LocalizationCamera camera, RobotVision vision) {
        boolean desiredUse = state.useForLocalizationEntry.getBoolean(camera.isUseForLocalization());
        if (desiredUse != camera.isUseForLocalization()) {
            if (vision != null) {
                vision.setUseForLocalization(state.key, desiredUse);
            } else {
                camera.setUseForLocalization(desiredUse);
            }
        }

        double currentTrust = camera.getConfig().getTrustDistance();
        double desiredTrust = state.trustDistanceEntry.getDouble(currentTrust);
        if (Math.abs(desiredTrust - currentTrust) > stdEpsilon) {
            camera.getConfig().setTrustDistance(Math.max(0.0, desiredTrust));
        }

        Matrix<N3, N1> singleStd = camera.getSingleStdDev();
        double desiredSingleX = sanitizeStdDev(state.singleStdXEntry.getDouble(singleStd.get(0, 0)));
        double desiredSingleY = sanitizeStdDev(state.singleStdYEntry.getDouble(singleStd.get(1, 0)));
        double desiredSingleThetaDeg =
                state.singleStdThetaDegEntry.getDouble(Math.toDegrees(singleStd.get(2, 0)));
        double desiredSingleThetaRad = Math.toRadians(desiredSingleThetaDeg);
        if (Math.abs(desiredSingleX - singleStd.get(0, 0)) > stdEpsilon
                || Math.abs(desiredSingleY - singleStd.get(1, 0)) > stdEpsilon
                || Math.abs(desiredSingleThetaRad - singleStd.get(2, 0)) > Math.toRadians(0.01)) {
            Matrix<N3, N1> updatedSingle = VecBuilder.fill(desiredSingleX, desiredSingleY, desiredSingleThetaRad);
            camera.getConfig().setSingleStdDevs(updatedSingle);
            camera.setStdDevs(updatedSingle, null);
        }

        Matrix<N3, N1> multiStd = camera.getMultiStdDev();
        double desiredMultiX = sanitizeStdDev(state.multiStdXEntry.getDouble(multiStd.get(0, 0)));
        double desiredMultiY = sanitizeStdDev(state.multiStdYEntry.getDouble(multiStd.get(1, 0)));
        double desiredMultiThetaDeg =
                state.multiStdThetaDegEntry.getDouble(Math.toDegrees(multiStd.get(2, 0)));
        double desiredMultiThetaRad = Math.toRadians(desiredMultiThetaDeg);
        if (Math.abs(desiredMultiX - multiStd.get(0, 0)) > stdEpsilon
                || Math.abs(desiredMultiY - multiStd.get(1, 0)) > stdEpsilon
                || Math.abs(desiredMultiThetaRad - multiStd.get(2, 0)) > Math.toRadians(0.01)) {
            Matrix<N3, N1> updatedMulti = VecBuilder.fill(desiredMultiX, desiredMultiY, desiredMultiThetaRad);
            camera.getConfig().setMultiStdDevs(updatedMulti);
            camera.setStdDevs(null, updatedMulti);
        }
    }

    private double sanitizeStdDev(double value) {
        double sanitized = Math.abs(value);
        if (!Double.isFinite(sanitized) || sanitized < stdEpsilon) {
            return stdEpsilon;
        }
        return sanitized;
    }

    private Pose2d computeCameraFieldPose(LocalizationCamera camera, Pose2d fieldPose) {
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

    private void updateEstimatedPose(CameraDisplayState state, LocalizationCamera camera) {
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

    private void updateTagLines(CameraDisplayState state, LocalizationCamera camera, Pose2d cameraPose, Pose2d fieldPose) {
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
        Pose2d tagPose = new Pose2d(targetFieldTranslation, new Rotation2d());
        state.visionTagLineObject.setPoses(cameraPose, tagPose);
    }

    private static class CameraDisplayState {
        final String key;
        final GenericEntry showCameraPoseEntry;
        final GenericEntry showTagLinesEntry;
        final GenericEntry showEstimatedPoseEntry;
        final GenericEntry useForLocalizationEntry;
        final GenericEntry trustDistanceEntry;
        final GenericEntry singleStdXEntry;
        final GenericEntry singleStdYEntry;
        final GenericEntry singleStdThetaDegEntry;
        final GenericEntry multiStdXEntry;
        final GenericEntry multiStdYEntry;
        final GenericEntry multiStdThetaDegEntry;
        final FieldObject2d visionCameraPoseObject;
        final FieldObject2d localizationEstimatedPoseObject;
        final FieldObject2d visionTagLineObject;
        final StructPublisher<Pose2d> cameraPosePublisher;
        final StructPublisher<Pose2d> estimatedPosePublisher;

        CameraDisplayState(
                String key,
                GenericEntry showCameraPoseEntry,
                GenericEntry showTagLinesEntry,
                GenericEntry showEstimatedPoseEntry,
                GenericEntry useForLocalizationEntry,
                GenericEntry trustDistanceEntry,
                GenericEntry singleStdXEntry,
                GenericEntry singleStdYEntry,
                GenericEntry singleStdThetaDegEntry,
                GenericEntry multiStdXEntry,
                GenericEntry multiStdYEntry,
                GenericEntry multiStdThetaDegEntry,
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
