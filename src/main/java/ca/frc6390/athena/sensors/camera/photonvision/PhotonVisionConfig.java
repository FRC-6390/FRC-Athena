package ca.frc6390.athena.sensors.camera.photonvision;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumSet;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.LocalizationCameraConfig.CameraRole;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;

public record PhotonVisionConfig(
        String table,
        Transform3d cameraRobotSpace,
        PoseStrategy poseStrategy,
        ArrayList<Integer> filteredTags,
        AprilTagFields fieldLayout,
        boolean useForLocalization,
        PoseStrategy poseStrategyFallback,
        double trustDistance,
        EnumSet<CameraRole> roles,
        double confidence,
        double simHorizontalFovDeg,
        double simVerticalFovDeg,
        int simResolutionWidth,
        int simResolutionHeight,
        int simMaxTargets) implements ConfigurableCamera {

    public PhotonVisionConfig {
        cameraRobotSpace = sanitizeTransform(cameraRobotSpace);
        filteredTags = filteredTags != null ? new ArrayList<>(filteredTags) : new ArrayList<>();
        roles = roles != null ? copyRoles(roles) : EnumSet.noneOf(CameraRole.class);
    }

    @Override
    public CameraSoftware getSoftware() {
        return CameraSoftware.PhotonVision;
    }

    public static PhotonVisionConfig table(String table){
        return new PhotonVisionConfig(
                table,
                new Transform3d(),
                PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                new ArrayList<>(),
                AprilTagFields.k2025ReefscapeWelded,
                false,
                PoseStrategy.LOWEST_AMBIGUITY,
                2,
                EnumSet.noneOf(CameraRole.class),
                1.0,
                70.0,
                43.0,
                640,
                480,
                10);
    }

    public PhotonVisionConfig setFieldLayout(AprilTagFields fieldLayout){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    public PhotonVisionConfig setCameraRobotSpace(Transform3d cameraRobotSpace){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    public PhotonVisionConfig setCameraRobotSpace(double x){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    public PhotonVisionConfig setTable(String table){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    public PhotonVisionConfig setPoseStrategy(PoseStrategy poseStrategy){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    public PhotonVisionConfig setMultiTagPoseStrategyFallback(PoseStrategy poseStrategyFallback){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    public PhotonVisionConfig setLocalizationTagFilter(Integer... filteredTags){
        var list = new ArrayList<>(Arrays.asList(filteredTags));
        list.add(-1);
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, list,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    public PhotonVisionConfig setUseForLocalization(boolean useForLocalization){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    public PhotonVisionConfig setTrustDistance(double trustDistance){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    public PhotonVisionConfig addRole(CameraRole role) {
        EnumSet<CameraRole> newRoles = copyRoles(roles);
        if (role != null) {
            newRoles.add(role);
        }
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags, fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, newRoles, confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    public PhotonVisionConfig setRoles(EnumSet<CameraRole> roles) {
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags, fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, roles != null ? copyRoles(roles) : EnumSet.noneOf(CameraRole.class), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    public PhotonVisionConfig setConfidence(double confidence) {
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags, fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    public PhotonVisionConfig setSimFov(double horizontalDeg, double verticalDeg) {
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags, fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, horizontalDeg, verticalDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    public PhotonVisionConfig setSimResolution(int width, int height) {
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags, fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, width, height, simMaxTargets);
    }

    public PhotonVisionConfig setSimMaxTargets(int maxTargets) {
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags, fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, maxTargets);
    }

    @Override
    public String getTable() {
       return table;
    }
    
    public PhotonVision create(){
        return new PhotonVision(table, cameraRobotSpace);
    }

    private static EnumSet<CameraRole> copyRoles(EnumSet<CameraRole> roles) {
        return roles == null || roles.isEmpty() ? EnumSet.noneOf(CameraRole.class) : EnumSet.copyOf(roles);
    }

    private static Transform3d sanitizeTransform(Transform3d transform) {
        if (transform == null) {
            return new Transform3d();
        }
        return new Transform3d(transform.getTranslation(), normalizeRotation(transform.getRotation()));
    }

    private static Rotation3d normalizeRotation(Rotation3d rotation) {
        if (rotation == null) {
            return new Rotation3d();
        }
        return new Rotation3d(
                normalizeAngle(rotation.getX()),
                normalizeAngle(rotation.getY()),
                normalizeAngle(rotation.getZ()));
    }

    private static double normalizeAngle(double angle) {
        double abs = Math.abs(angle);
        if (abs > (Math.PI * 2)) {
            return Math.toRadians(angle);
        }
        return angle;
    }
}
