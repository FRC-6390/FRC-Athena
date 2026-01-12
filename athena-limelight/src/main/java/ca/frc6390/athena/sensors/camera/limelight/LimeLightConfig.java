package ca.frc6390.athena.sensors.camera.limelight;


import java.util.Arrays;
import java.util.EnumSet;

import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.LocalizationCameraConfig.CameraRole;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

/**
 * Immutable configuration for a Limelight camera. The record exposes builder-like helpers that
 * produce modified copies with updated parameters.
 */
public record LimeLightConfig(
        String table,
        Transform3d cameraRobotSpace,
        PoseEstimateWithLatencyType localizationEstimator,
        int[] filteredTags,
        AprilTagFields fieldLayout,
        boolean useForLocalization,
        double trustDistance,
        EnumSet<CameraRole> roles,
        double confidence) implements ConfigurableCamera {
    private static final String DEFUALT_TABLE = "limelight"; 

    public LimeLightConfig {
        cameraRobotSpace = sanitizeTransform(cameraRobotSpace);
        filteredTags = filteredTags != null ? Arrays.copyOf(filteredTags, filteredTags.length) : new int[]{};
        roles = roles != null ? copyRoles(roles) : EnumSet.noneOf(CameraRole.class);
        fieldLayout = fieldLayout != null ? fieldLayout : AprilTagFields.k2025ReefscapeWelded;
    }

    /**
     * Creates a configuration pointing to the specified NetworkTables table.
     */
    public LimeLightConfig(String table){
        this(
                table,
                new Transform3d(),
                PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE,
                new int[]{},
                AprilTagFields.k2025ReefscapeWelded,
                false,
                2,
                EnumSet.noneOf(CameraRole.class),
                1.0);
    }

    /**
     * Creates a configuration using the default {@code "limelight"} table.
     */
    public LimeLightConfig(){
        this(DEFUALT_TABLE);
    }

    /**
     * Convenience factory for {@link #LimeLightConfig(String)}.
     */
    public static LimeLightConfig table(String table){
        return new LimeLightConfig(table);
    }

    /**
     * Returns a copy of this configuration with an updated camera transform.
     */
    public LimeLightConfig setCameraTransform(Transform3d cameraRobotSpace){
        return new LimeLightConfig(
                table,
                cameraRobotSpace,
                localizationEstimator,
                copyTags(filteredTags),
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence);
    }

    /**
     * Returns a copy of this configuration with a different pose estimation strategy.
     */
    public LimeLightConfig setPoseEstimateType(PoseEstimateWithLatencyType localizationEstimator){
        return new LimeLightConfig(
                table,
                cameraRobotSpace,
                localizationEstimator,
                copyTags(filteredTags),
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence);
    }

    /**
     * Returns a copy of this configuration with a filtered tag list used for localization.
     */
    public LimeLightConfig setLocalizationTagFilter(int... filteredTags){
        return new LimeLightConfig(
                table,
                cameraRobotSpace,
                localizationEstimator,
                filteredTags != null ? Arrays.copyOf(filteredTags, filteredTags.length) : new int[]{},
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence);
    }

    /**
     * Returns a copy with an updated underlying AprilTag field layout.
     */
    public LimeLightConfig setFieldLayout(AprilTagFields fieldLayout) {
        return new LimeLightConfig(
                table,
                cameraRobotSpace,
                localizationEstimator,
                copyTags(filteredTags),
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence);
    }
    
    /**
     * Returns a copy that toggles whether Limelight measurements feed localization.
     */
    public LimeLightConfig setUseForLocalization(boolean useForLocalization){
        return new LimeLightConfig(
                table,
                cameraRobotSpace,
                localizationEstimator,
                copyTags(filteredTags),
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence);
    }

    /**
     * Returns a copy with an updated trust distance (meters).
     */
    public LimeLightConfig setTrustDistance(double trustDistance){
        return new LimeLightConfig(
                table,
                cameraRobotSpace,
                localizationEstimator,
                copyTags(filteredTags),
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence);
    }

    /**
     * Returns a copy with the provided role appended to the set.
     */
    public LimeLightConfig addRole(CameraRole role) {
        EnumSet<CameraRole> newRoles = copyRoles(roles);
        if (role != null) {
            newRoles.add(role);
        }
        return new LimeLightConfig(
                table,
                cameraRobotSpace,
                localizationEstimator,
                copyTags(filteredTags),
                fieldLayout,
                useForLocalization,
                trustDistance,
                newRoles,
                confidence);
    }

    /**
     * Returns a copy with an entirely new set of roles.
     */
    public LimeLightConfig setRoles(EnumSet<CameraRole> newRoles) {
        EnumSet<CameraRole> copy = newRoles != null ? copyRoles(newRoles) : EnumSet.noneOf(CameraRole.class);
        return new LimeLightConfig(
                table,
                cameraRobotSpace,
                localizationEstimator,
                copyTags(filteredTags),
                fieldLayout,
                useForLocalization,
                trustDistance,
                copy,
                confidence);
    }

    /**
     * Returns a copy with an updated pipeline confidence [0, 1].
     */
    public LimeLightConfig setConfidence(double confidence) {
        return new LimeLightConfig(
                table,
                cameraRobotSpace,
                localizationEstimator,
                copyTags(filteredTags),
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence);
    }

    @Override
    public CameraSoftware getSoftware() {
        return CameraSoftware.LimeLight;
    }

    @Override
    public String getTable() {
        return table;
    }

    /**
     * Returns a copy with the robot-space camera transform replaced.
     */
    public LimeLightConfig setCameraRobotSpace(Transform3d transform) {
        return new LimeLightConfig(
                table,
                transform,
                localizationEstimator,
                copyTags(filteredTags),
                fieldLayout,
                useForLocalization,
                trustDistance,
                copyRoles(roles),
                confidence);
    }

    private static EnumSet<CameraRole> copyRoles(EnumSet<CameraRole> roles) {
        return roles == null || roles.isEmpty() ? EnumSet.noneOf(CameraRole.class) : EnumSet.copyOf(roles);
    }

    private static int[] copyTags(int[] tags) {
        return tags == null ? new int[]{} : Arrays.copyOf(tags, tags.length);
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
