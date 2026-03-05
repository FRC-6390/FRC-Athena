package ca.frc6390.athena.sensors.camera.helios;

import java.util.EnumSet;

import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.VisionCameraConfig.CameraRole;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Immutable configuration for a HeliOS-backed camera.
 */
public record HeliOSConfig(
        String table,
        String target,
        String localizationProfile,
        String solverId,
        Transform3d cameraRobotSpace,
        AprilTagFields fieldLayout,
        boolean useForLocalization,
        double trustDistance,
        EnumSet<CameraRole> roles,
        double confidence,
        double simHorizontalFovDeg,
        double simVerticalFovDeg,
        int simResolutionWidth,
        int simResolutionHeight,
        int simMaxTargets) implements ConfigurableCamera {

    private static final String DEFAULT_TABLE = "helios";
    private static final String DEFAULT_TARGET = "helios.local";

    public HeliOSConfig {
        table = sanitizeTable(table);
        target = sanitizeTarget(target);
        localizationProfile = sanitizeOptional(localizationProfile);
        solverId = sanitizeOptional(solverId);
        cameraRobotSpace = sanitizeTransform(cameraRobotSpace);
        fieldLayout = fieldLayout != null ? fieldLayout : AprilTagFields.kDefaultField;
        roles = roles != null ? copyRoles(roles) : EnumSet.noneOf(CameraRole.class);
    }

    public static HeliOSConfig table(String table, String target) {
        return new HeliOSConfig(
                table,
                target,
                null,
                null,
                new Transform3d(),
                AprilTagFields.kDefaultField,
                false,
                2.0,
                EnumSet.noneOf(CameraRole.class),
                1.0,
                70.0,
                43.0,
                640,
                480,
                10);
    }

    public static HeliOSConfig target(String target) {
        return table(DEFAULT_TABLE, target);
    }

    public HeliOSConfig withTable(String table) {
        return new HeliOSConfig(
                table,
                target,
                localizationProfile,
                solverId,
                cameraRobotSpace,
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence,
                simHorizontalFovDeg,
                simVerticalFovDeg,
                simResolutionWidth,
                simResolutionHeight,
                simMaxTargets);
    }

    public HeliOSConfig withTarget(String target) {
        return new HeliOSConfig(
                table,
                target,
                localizationProfile,
                solverId,
                cameraRobotSpace,
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence,
                simHorizontalFovDeg,
                simVerticalFovDeg,
                simResolutionWidth,
                simResolutionHeight,
                simMaxTargets);
    }

    public HeliOSConfig withLocalizationProfile(String localizationProfile) {
        return new HeliOSConfig(
                table,
                target,
                localizationProfile,
                solverId,
                cameraRobotSpace,
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence,
                simHorizontalFovDeg,
                simVerticalFovDeg,
                simResolutionWidth,
                simResolutionHeight,
                simMaxTargets);
    }

    public HeliOSConfig withSolverId(String solverId) {
        return new HeliOSConfig(
                table,
                target,
                localizationProfile,
                solverId,
                cameraRobotSpace,
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence,
                simHorizontalFovDeg,
                simVerticalFovDeg,
                simResolutionWidth,
                simResolutionHeight,
                simMaxTargets);
    }

    public HeliOSConfig withFieldLayout(AprilTagFields fieldLayout) {
        return new HeliOSConfig(
                table,
                target,
                localizationProfile,
                solverId,
                cameraRobotSpace,
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence,
                simHorizontalFovDeg,
                simVerticalFovDeg,
                simResolutionWidth,
                simResolutionHeight,
                simMaxTargets);
    }

    public HeliOSConfig withCameraRobotSpace(Transform3d cameraRobotSpace) {
        return new HeliOSConfig(
                table,
                target,
                localizationProfile,
                solverId,
                cameraRobotSpace,
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence,
                simHorizontalFovDeg,
                simVerticalFovDeg,
                simResolutionWidth,
                simResolutionHeight,
                simMaxTargets);
    }

    public HeliOSConfig withUseForLocalization(boolean useForLocalization) {
        return new HeliOSConfig(
                table,
                target,
                localizationProfile,
                solverId,
                cameraRobotSpace,
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence,
                simHorizontalFovDeg,
                simVerticalFovDeg,
                simResolutionWidth,
                simResolutionHeight,
                simMaxTargets);
    }

    public HeliOSConfig withTrustDistance(double trustDistance) {
        return new HeliOSConfig(
                table,
                target,
                localizationProfile,
                solverId,
                cameraRobotSpace,
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence,
                simHorizontalFovDeg,
                simVerticalFovDeg,
                simResolutionWidth,
                simResolutionHeight,
                simMaxTargets);
    }

    public HeliOSConfig addRole(CameraRole role) {
        EnumSet<CameraRole> newRoles = copyRoles(roles);
        if (role != null) {
            newRoles.add(role);
        }
        return new HeliOSConfig(
                table,
                target,
                localizationProfile,
                solverId,
                cameraRobotSpace,
                fieldLayout,
                useForLocalization,
                trustDistance,
                newRoles,
                confidence,
                simHorizontalFovDeg,
                simVerticalFovDeg,
                simResolutionWidth,
                simResolutionHeight,
                simMaxTargets);
    }

    public HeliOSConfig withRoles(EnumSet<CameraRole> roles) {
        return new HeliOSConfig(
                table,
                target,
                localizationProfile,
                solverId,
                cameraRobotSpace,
                fieldLayout,
                useForLocalization,
                trustDistance,
                roles != null ? copyRoles(roles) : EnumSet.noneOf(CameraRole.class),
                confidence,
                simHorizontalFovDeg,
                simVerticalFovDeg,
                simResolutionWidth,
                simResolutionHeight,
                simMaxTargets);
    }

    public HeliOSConfig withConfidence(double confidence) {
        return new HeliOSConfig(
                table,
                target,
                localizationProfile,
                solverId,
                cameraRobotSpace,
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence,
                simHorizontalFovDeg,
                simVerticalFovDeg,
                simResolutionWidth,
                simResolutionHeight,
                simMaxTargets);
    }

    public HeliOSConfig withSimFov(double horizontalDeg, double verticalDeg) {
        return new HeliOSConfig(
                table,
                target,
                localizationProfile,
                solverId,
                cameraRobotSpace,
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence,
                horizontalDeg,
                verticalDeg,
                simResolutionWidth,
                simResolutionHeight,
                simMaxTargets);
    }

    public HeliOSConfig withSimResolution(int width, int height) {
        return new HeliOSConfig(
                table,
                target,
                localizationProfile,
                solverId,
                cameraRobotSpace,
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence,
                simHorizontalFovDeg,
                simVerticalFovDeg,
                width,
                height,
                simMaxTargets);
    }

    public HeliOSConfig withSimMaxTargets(int maxTargets) {
        return new HeliOSConfig(
                table,
                target,
                localizationProfile,
                solverId,
                cameraRobotSpace,
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence,
                simHorizontalFovDeg,
                simVerticalFovDeg,
                simResolutionWidth,
                simResolutionHeight,
                maxTargets);
    }

    public HeliOSCameraAdapter create() {
        return new HeliOSCameraAdapter(this);
    }

    @Override
    public CameraSoftware getSoftware() {
        return CameraSoftware.HeliOS;
    }

    @Override
    public String getTable() {
        return table;
    }

    private static EnumSet<CameraRole> copyRoles(EnumSet<CameraRole> roles) {
        return roles == null || roles.isEmpty() ? EnumSet.noneOf(CameraRole.class) : EnumSet.copyOf(roles);
    }

    private static String sanitizeTable(String table) {
        String value = sanitizeOptional(table);
        return value != null ? value : DEFAULT_TABLE;
    }

    private static String sanitizeTarget(String target) {
        String value = sanitizeOptional(target);
        return value != null ? value : DEFAULT_TARGET;
    }

    private static String sanitizeOptional(String value) {
        if (value == null) {
            return null;
        }
        String trimmed = value.trim();
        return trimmed.isEmpty() ? null : trimmed;
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
