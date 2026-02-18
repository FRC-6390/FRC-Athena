package ca.frc6390.athena.core.localization;

import java.util.ArrayList;
import java.util.ArrayDeque;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.function.Supplier;

import ca.frc6390.athena.core.RobotAuto;
import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

final class RobotLocalizationFieldPublisher {
    private static final Pose2d ZERO_POSE = new Pose2d();
    private static final Pose2d[] EMPTY_POSE_ARRAY = new Pose2d[0];
    private static final List<Pose2d> EMPTY_POSES = List.of();
    private static final Rotation2d ZERO_ROTATION = Rotation2d.kZero;
    private static final String AUTO_NT_ROOT = "Athena/Auto";

    private final Field2d field;
    private final Supplier<RobotVision> visionSupplier;
    private final StructArrayPublisher<Pose2d> plannedPathPublisher;
    private final StructArrayPublisher<Pose2d> localizationPathPublisher;
    private final StringPublisher selectedAutoIdPublisher;
    private FieldObject2d plannedPathObject;
    private FieldObject2d actualPathObject;
    private final Map<String, FieldObject2d> boundingBoxObjects = new HashMap<>();
    private final Map<String, FieldObject2d> namedPoseObjects = new HashMap<>();
    private final Map<String, List<Pose2d>> boundingBoxPathCache = new HashMap<>();
    private final Map<String, PoseBoundingBox2d> boundingBoxPathSources = new HashMap<>();
    private final ArrayDeque<Pose2d> actualPathPoses = new ArrayDeque<>();
    private final List<Pose2d> actualPathPoseSnapshot = new ArrayList<>();
    private double actualPathSpacingMeters = 0.1;
    private double actualPathMinIntervalSeconds = 0.1;
    private int actualPathMaxPoints = 600;
    private double plannedPathSpacingMeters = 0.2;
    private Double fieldLengthMetersOverride = null;
    private Double fieldWidthMetersOverride = null;
    private double lastActualPathTimestamp = Double.NEGATIVE_INFINITY;
    private Pose2d lastActualPathPose = ZERO_POSE;
    private String lastPlannedAutoId;
    private boolean plannedPathEmpty = true;
    private boolean fieldPublished;

    RobotLocalizationFieldPublisher(Supplier<RobotVision> visionSupplier) {
        this.field = new Field2d();
        this.visionSupplier = visionSupplier;
        this.plannedPathPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic(AUTO_NT_ROOT + "/PlannedPath", Pose2d.struct)
                .publish();
        this.localizationPathPublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic(AUTO_NT_ROOT + "/LocalizationPath", Pose2d.struct)
                .publish();
        this.selectedAutoIdPublisher = NetworkTableInstance.getDefault()
                .getStringTopic(AUTO_NT_ROOT + "/SelectedRoutine")
                .publish();
        this.plannedPathObject = field.getObject("AutoPlan");
        this.actualPathObject = field.getObject("ActualPath");
    }

    Field2d getField() {
        return field;
    }

    FieldObject2d getObject(String id) {
        return field.getObject(id);
    }

    void setRobotPose(Pose2d pose) {
        field.setRobotPose(pose);
    }

    void publishFieldOnce() {
        fieldPublished = true;
    }

    boolean isFieldPublished() {
        return fieldPublished;
    }

    void clearRobotPose() {
        field.getObject("Robot").setPoses(EMPTY_POSES);
    }

    void setNamedPoses(Map<String, Pose2d> poses) {
        if (poses == null || poses.isEmpty()) {
            clearNamedPoses();
            return;
        }

        namedPoseObjects.entrySet().removeIf(entry -> {
            if (poses.containsKey(entry.getKey())) {
                return false;
            }
            entry.getValue().setPoses(EMPTY_POSES);
            return true;
        });

        for (Map.Entry<String, Pose2d> entry : poses.entrySet()) {
            String key = entry.getKey();
            Pose2d pose = entry.getValue();
            if (key == null || key.isBlank() || pose == null) {
                continue;
            }
            FieldObject2d object = namedPoseObjects.computeIfAbsent(
                    key,
                    id -> field.getObject("Poses/" + id));
            object.setPose(pose);
        }
    }

    void clearNamedPoses() {
        for (FieldObject2d object : namedPoseObjects.values()) {
            object.setPoses(EMPTY_POSES);
        }
        namedPoseObjects.clear();
    }

    void setBoundingBoxes(Map<String, PoseBoundingBox2d> boxes) {
        if (boxes == null || boxes.isEmpty()) {
            clearBoundingBoxes();
            return;
        }

        boundingBoxObjects.entrySet().removeIf(entry -> {
            if (boxes.containsKey(entry.getKey())) {
                return false;
            }
            entry.getValue().setPoses(EMPTY_POSES);
            boundingBoxPathCache.remove(entry.getKey());
            boundingBoxPathSources.remove(entry.getKey());
            return true;
        });

        for (Map.Entry<String, PoseBoundingBox2d> entry : boxes.entrySet()) {
            String key = entry.getKey();
            PoseBoundingBox2d box = entry.getValue();
            if (key == null || key.isBlank() || box == null) {
                continue;
            }
            FieldObject2d object = boundingBoxObjects.computeIfAbsent(
                    key,
                    id -> field.getObject("BoundingBoxes/" + id));
            object.setPoses(getOrBuildBoundingBoxPath(key, box));
        }
    }

    void clearBoundingBoxes() {
        for (FieldObject2d object : boundingBoxObjects.values()) {
            object.setPoses(EMPTY_POSES);
        }
        boundingBoxObjects.clear();
        boundingBoxPathCache.clear();
        boundingBoxPathSources.clear();
    }

    void updateAutoVisualization(RobotAuto autos) {
        if (autos == null) {
            selectedAutoIdPublisher.set("");
            clearPlannedPath();
            return;
        }
        String selectedId = autos.selection().selected()
                .map(routine -> routine.key().id())
                .orElse(null);
        selectedAutoIdPublisher.set(selectedId != null ? selectedId : "");
        if (Objects.equals(selectedId, lastPlannedAutoId) && !isPlannedPathEmpty()) {
            return;
        }
        lastPlannedAutoId = selectedId;
        autos.selection().selectedPoses()
                .filter(list -> !list.isEmpty())
                .ifPresentOrElse(this::setPlannedPath, this::clearPlannedPath);
    }

    void setPlannedPath(List<Pose2d> poses) {
        if (plannedPathObject == null) {
            plannedPathObject = field.getObject("AutoPlan");
        }
        if (poses == null || poses.isEmpty()) {
            plannedPathObject.setPoses(EMPTY_POSES);
            plannedPathPublisher.set(EMPTY_POSE_ARRAY);
            plannedPathEmpty = true;
            return;
        }
        List<Pose2d> resolved = applyAllianceFlip(poses);
        resolved = downsamplePoses(resolved, plannedPathSpacingMeters);
        if (resolved.isEmpty()) {
            plannedPathObject.setPoses(EMPTY_POSES);
            plannedPathPublisher.set(EMPTY_POSE_ARRAY);
            plannedPathEmpty = true;
            return;
        }
        plannedPathObject.setPoses(resolved);
        plannedPathPublisher.set(resolved.toArray(EMPTY_POSE_ARRAY));
        plannedPathEmpty = false;
    }

    void clearPlannedPath() {
        if (plannedPathObject == null) {
            plannedPathObject = field.getObject("AutoPlan");
        }
        plannedPathObject.setPoses(EMPTY_POSES);
        plannedPathPublisher.set(EMPTY_POSE_ARRAY);
        plannedPathEmpty = true;
    }

    void resetActualPath() {
        actualPathPoses.clear();
        actualPathPoseSnapshot.clear();
        lastActualPathPose = ZERO_POSE;
        lastActualPathTimestamp = Double.NEGATIVE_INFINITY;
        if (actualPathObject == null) {
            actualPathObject = field.getObject("ActualPath");
        }
        actualPathObject.setPoses(EMPTY_POSES);
        localizationPathPublisher.set(EMPTY_POSE_ARRAY);
    }

    void setPlannedPathSpacingMeters(double spacingMeters) {
        if (Double.isFinite(spacingMeters) && spacingMeters > 0.0) {
            plannedPathSpacingMeters = spacingMeters;
        }
    }

    void setFieldDimensionsMeters(double lengthMeters, double widthMeters) {
        if (Double.isFinite(lengthMeters) && lengthMeters > 0.0) {
            fieldLengthMetersOverride = lengthMeters;
        }
        if (Double.isFinite(widthMeters) && widthMeters > 0.0) {
            fieldWidthMetersOverride = widthMeters;
        }
    }

    void setActualPathSpacingMeters(double spacingMeters) {
        if (Double.isFinite(spacingMeters) && spacingMeters > 0.0) {
            actualPathSpacingMeters = spacingMeters;
        }
    }

    void setActualPathMinIntervalSeconds(double minIntervalSeconds) {
        if (Double.isFinite(minIntervalSeconds) && minIntervalSeconds >= 0.0) {
            actualPathMinIntervalSeconds = minIntervalSeconds;
        }
    }

    void setActualPathMaxPoints(int maxPoints) {
        if (maxPoints > 0) {
            actualPathMaxPoints = maxPoints;
        }
    }

    void updateActualPath(Pose2d pose) {
        if (pose == null) {
            return;
        }
        double now = Timer.getFPGATimestamp();
        if (actualPathPoses.isEmpty()) {
            appendActualPose(pose, now);
            return;
        }
        double distance = pose.getTranslation().getDistance(lastActualPathPose.getTranslation());
        if (distance < actualPathSpacingMeters
                && now - lastActualPathTimestamp < actualPathMinIntervalSeconds) {
            return;
        }
        appendActualPose(pose, now);
    }

    private void appendActualPose(Pose2d pose, double timestampSeconds) {
        if (actualPathObject == null) {
            actualPathObject = field.getObject("ActualPath");
        }
        actualPathPoses.addLast(pose);
        boolean pruned = false;
        while (actualPathPoses.size() > actualPathMaxPoints) {
            actualPathPoses.removeFirst();
            pruned = true;
        }
        lastActualPathPose = pose;
        lastActualPathTimestamp = timestampSeconds;
        if (pruned) {
            actualPathPoseSnapshot.clear();
            actualPathPoseSnapshot.addAll(actualPathPoses);
        } else {
            actualPathPoseSnapshot.add(pose);
        }
        actualPathObject.setPoses(actualPathPoseSnapshot);
        localizationPathPublisher.set(actualPathPoseSnapshot.toArray(EMPTY_POSE_ARRAY));
    }

    private boolean isPlannedPathEmpty() {
        return plannedPathEmpty;
    }

    private List<Pose2d> applyAllianceFlip(List<Pose2d> poses) {
        if (poses == null || poses.isEmpty()) {
            return List.of();
        }
        boolean isRed =
                DriverStation.getAlliance().map(a -> a == DriverStation.Alliance.Red).orElse(false);
        if (!isRed) {
            return poses;
        }
        double fieldLength = resolveFieldLengthMeters();
        double fieldWidth = resolveFieldWidthMeters();
        List<Pose2d> flipped = new ArrayList<>(poses.size());
        for (Pose2d pose : poses) {
            if (pose == null) {
                continue;
            }
            double x = fieldLength - pose.getX();
            double y = fieldWidth - pose.getY();
            Rotation2d rotation = pose.getRotation().plus(Rotation2d.fromDegrees(180.0));
            flipped.add(new Pose2d(x, y, rotation));
        }
        return flipped;
    }

    private double resolveFieldLengthMeters() {
        if (fieldLengthMetersOverride != null) {
            return fieldLengthMetersOverride;
        }
        RobotVision vision = visionSupplier != null ? visionSupplier.get() : null;
        if (vision != null) {
            for (VisionCamera camera : vision.cameras().all().values()) {
                var layout = camera != null ? camera.getConfig().getFieldLayout() : null;
                if (layout != null) {
                    return layout.getFieldLength();
                }
            }
        }
        return 16.54;
    }

    private double resolveFieldWidthMeters() {
        if (fieldWidthMetersOverride != null) {
            return fieldWidthMetersOverride;
        }
        RobotVision vision = visionSupplier != null ? visionSupplier.get() : null;
        if (vision != null) {
            for (VisionCamera camera : vision.cameras().all().values()) {
                var layout = camera != null ? camera.getConfig().getFieldLayout() : null;
                if (layout != null) {
                    return layout.getFieldWidth();
                }
            }
        }
        return 8.21;
    }

    private static List<Pose2d> downsamplePoses(List<Pose2d> poses, double spacingMeters) {
        if (poses == null || poses.isEmpty()) {
            return List.of();
        }
        if (!Double.isFinite(spacingMeters) || spacingMeters <= 0.0) {
            return poses;
        }
        List<Pose2d> result = new ArrayList<>();
        Pose2d last = null;
        for (Pose2d pose : poses) {
            if (pose == null) {
                continue;
            }
            if (last == null
                    || pose.getTranslation().getDistance(last.getTranslation()) >= spacingMeters) {
                result.add(pose);
                last = pose;
            }
        }
        return result;
    }

    private List<Pose2d> getOrBuildBoundingBoxPath(String name, PoseBoundingBox2d box) {
        PoseBoundingBox2d previous = boundingBoxPathSources.get(name);
        List<Pose2d> cached = boundingBoxPathCache.get(name);
        if (cached != null && box.equals(previous)) {
            return cached;
        }
        List<Pose2d> rebuilt = toBoundingBoxPath(box);
        boundingBoxPathCache.put(name, rebuilt);
        boundingBoxPathSources.put(name, box);
        return rebuilt;
    }

    private static List<Pose2d> toBoundingBoxPath(PoseBoundingBox2d box) {
        Pose2d bottomLeft = new Pose2d(box.minX(), box.minY(), ZERO_ROTATION);
        Pose2d bottomRight = new Pose2d(box.maxX(), box.minY(), ZERO_ROTATION);
        Pose2d topRight = new Pose2d(box.maxX(), box.maxY(), ZERO_ROTATION);
        Pose2d topLeft = new Pose2d(box.minX(), box.maxY(), ZERO_ROTATION);
        return List.of(bottomLeft, bottomRight, topRight, topLeft, bottomLeft);
    }
}
