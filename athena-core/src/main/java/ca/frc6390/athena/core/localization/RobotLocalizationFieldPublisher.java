package ca.frc6390.athena.core.localization;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;

import ca.frc6390.athena.core.RobotAuto;
import ca.frc6390.athena.core.RobotVision;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

final class RobotLocalizationFieldPublisher {
    private final Field2d field;
    private final Supplier<RobotVision> visionSupplier;
    private FieldObject2d plannedPathObject;
    private FieldObject2d actualPathObject;
    private final List<Pose2d> actualPathPoses = new ArrayList<>();
    private double actualPathSpacingMeters = 0.1;
    private double actualPathMinIntervalSeconds = 0.1;
    private int actualPathMaxPoints = 600;
    private double plannedPathSpacingMeters = 0.2;
    private Double fieldLengthMetersOverride = null;
    private Double fieldWidthMetersOverride = null;
    private double lastActualPathTimestamp = Double.NEGATIVE_INFINITY;
    private Pose2d lastActualPathPose = new Pose2d();
    private String lastPlannedAutoId;
    private boolean fieldPublished;

    RobotLocalizationFieldPublisher(Supplier<RobotVision> visionSupplier) {
        this.field = new Field2d();
        this.visionSupplier = visionSupplier;
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
        field.getObject("Robot").setPoses(List.of());
    }

    void updateAutoVisualization(RobotAuto autos) {
        if (autos == null) {
            clearPlannedPath();
            return;
        }
        String selectedId = autos.getSelectedAuto()
                .map(routine -> routine.key().id())
                .orElse(null);
        if (Objects.equals(selectedId, lastPlannedAutoId) && !isPlannedPathEmpty()) {
            return;
        }
        lastPlannedAutoId = selectedId;
        autos.getSelectedAutoPoses()
                .filter(list -> !list.isEmpty())
                .ifPresentOrElse(this::setPlannedPath, this::clearPlannedPath);
    }

    void setPlannedPath(List<Pose2d> poses) {
        if (plannedPathObject == null) {
            plannedPathObject = field.getObject("AutoPlan");
        }
        if (poses == null || poses.isEmpty()) {
            plannedPathObject.setPoses(List.of());
            return;
        }
        List<Pose2d> resolved = applyAllianceFlip(poses);
        resolved = downsamplePoses(resolved, plannedPathSpacingMeters);
        plannedPathObject.setPoses(resolved);
    }

    void clearPlannedPath() {
        if (plannedPathObject == null) {
            plannedPathObject = field.getObject("AutoPlan");
        }
        plannedPathObject.setPoses(List.of());
    }

    void resetActualPath() {
        actualPathPoses.clear();
        lastActualPathPose = new Pose2d();
        lastActualPathTimestamp = Double.NEGATIVE_INFINITY;
        if (actualPathObject == null) {
            actualPathObject = field.getObject("ActualPath");
        }
        actualPathObject.setPoses(List.of());
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
        actualPathPoses.add(pose);
        if (actualPathPoses.size() > actualPathMaxPoints) {
            actualPathPoses.remove(0);
        }
        lastActualPathPose = pose;
        lastActualPathTimestamp = timestampSeconds;
        actualPathObject.setPoses(actualPathPoses);
    }

    private boolean isPlannedPathEmpty() {
        return plannedPathObject == null
                || plannedPathObject.getPoses() == null
                || plannedPathObject.getPoses().isEmpty();
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
            for (VisionCamera camera : vision.getCameras().values()) {
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
            for (VisionCamera camera : vision.getCameras().values()) {
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
}
