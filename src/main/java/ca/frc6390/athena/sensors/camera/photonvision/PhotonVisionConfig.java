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

/**
 * Immutable configuration for a PhotonVision camera. The record behaves like a builder by returning
 * modified copies through its setter-style methods.
 */
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

    /**
     * Creates a default PhotonVision configuration for the provided table.
     */
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

    /**
     * Returns a copy with a different AprilTag field layout.
     */
    public PhotonVisionConfig setFieldLayout(AprilTagFields fieldLayout){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    /**
     * Returns a copy with an updated camera transform.
     */
    public PhotonVisionConfig setCameraRobotSpace(Transform3d cameraRobotSpace){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    /**
     * Returns a copy with the same transform (legacy helper retained for compatibility).
     */
    public PhotonVisionConfig setCameraRobotSpace(double x){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    /**
     * Returns a copy using the provided NetworkTables table.
     */
    public PhotonVisionConfig setTable(String table){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    /**
     * Returns a copy with a new primary pose strategy.
     */
    public PhotonVisionConfig setPoseStrategy(PoseStrategy poseStrategy){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    /**
     * Returns a copy with a fallback pose strategy used when multi-tag estimates fail.
     */
    public PhotonVisionConfig setMultiTagPoseStrategyFallback(PoseStrategy poseStrategyFallback){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    /**
     * Returns a copy with a filtered list of tags used for localization.
     */
    public PhotonVisionConfig setLocalizationTagFilter(Integer... filteredTags){
        var list = new ArrayList<>(Arrays.asList(filteredTags));
        list.add(-1);
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, list,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    /**
     * Returns a copy toggling whether PhotonVision contributes to localization.
     */
    public PhotonVisionConfig setUseForLocalization(boolean useForLocalization){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    /**
     * Returns a copy with an updated trust distance (meters).
     */
    public PhotonVisionConfig setTrustDistance(double trustDistance){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags,fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    /**
     * Returns a copy with the provided role appended to the set.
     */
    public PhotonVisionConfig addRole(CameraRole role) {
        EnumSet<CameraRole> newRoles = copyRoles(roles);
        if (role != null) {
            newRoles.add(role);
        }
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags, fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, newRoles, confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    /**
     * Returns a copy with an entirely new set of roles.
     */
    public PhotonVisionConfig setRoles(EnumSet<CameraRole> roles) {
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags, fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, roles != null ? copyRoles(roles) : EnumSet.noneOf(CameraRole.class), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    /**
     * Returns a copy with an updated pipeline confidence [0, 1].
     */
    public PhotonVisionConfig setConfidence(double confidence) {
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags, fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    /**
     * Returns a copy with updated simulated field-of-view angles (degrees).
     */
    public PhotonVisionConfig setSimFov(double horizontalDeg, double verticalDeg) {
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags, fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, horizontalDeg, verticalDeg, simResolutionWidth, simResolutionHeight, simMaxTargets);
    }

    /**
     * Returns a copy with updated simulated camera resolution (pixels).
     */
    public PhotonVisionConfig setSimResolution(int width, int height) {
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags, fieldLayout, useForLocalization, poseStrategyFallback, trustDistance, copyRoles(roles), confidence, simHorizontalFovDeg, simVerticalFovDeg, width, height, simMaxTargets);
    }

    /**
     * Returns a copy with an updated maximum number of simulated targets per frame.
     */
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
