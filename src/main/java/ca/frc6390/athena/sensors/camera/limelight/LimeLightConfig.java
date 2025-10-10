package ca.frc6390.athena.sensors.camera.limelight;


import java.util.Arrays;
import java.util.EnumSet;

import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.LocalizationCameraConfig.CameraRole;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import ca.frc6390.athena.sensors.camera.photonvision.PhotonVisionConfig;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public record LimeLightConfig(
        String table,
        Transform3d cameraRobotSpace,
        PoseEstimateWithLatencyType localizationEstimator,
        int[] filteredTags,
        boolean useForLocalization,
        double trustDistance,
        EnumSet<CameraRole> roles,
        double confidence) implements ConfigurableCamera {
    private static final String DEFUALT_TABLE = "limelight"; 

    public LimeLightConfig {
        cameraRobotSpace = sanitizeTransform(cameraRobotSpace);
        filteredTags = filteredTags != null ? Arrays.copyOf(filteredTags, filteredTags.length) : new int[]{};
        roles = roles != null ? copyRoles(roles) : EnumSet.noneOf(CameraRole.class);
    }

    public LimeLightConfig(String table){
        this(table, new Transform3d(), PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE, new int[]{}, false, 2, EnumSet.noneOf(CameraRole.class), 1.0);
    }

    public LimeLightConfig(){
        this(DEFUALT_TABLE);
    }

    public static LimeLightConfig table(String table){
        return new LimeLightConfig(table);
    }

    public LimeLightConfig setCameraTransform(Transform3d cameraRobotSpace){
        return new LimeLightConfig(table, cameraRobotSpace, localizationEstimator, copyTags(filteredTags), useForLocalization, trustDistance, copyRoles(roles), confidence);
    }

    public LimeLightConfig setPoseEstimateType(PoseEstimateWithLatencyType localizationEstimator){
        return new LimeLightConfig(table, cameraRobotSpace, localizationEstimator, copyTags(filteredTags), useForLocalization, trustDistance, copyRoles(roles), confidence);
    }

    public LimeLightConfig setLocalizationTagFilter(int... filteredTags){
        return new LimeLightConfig(table, cameraRobotSpace, localizationEstimator, filteredTags != null ? Arrays.copyOf(filteredTags, filteredTags.length) : new int[]{}, useForLocalization, trustDistance, copyRoles(roles), confidence);
    }
    
    public LimeLightConfig setUseForLocalization(boolean useForLocalization){
        return new LimeLightConfig(table, cameraRobotSpace, localizationEstimator, copyTags(filteredTags), useForLocalization, trustDistance, copyRoles(roles), confidence);
    }

    public LimeLightConfig setTrustDistance(double trustDistance){
        return new LimeLightConfig(table, cameraRobotSpace, localizationEstimator, copyTags(filteredTags), useForLocalization, trustDistance, copyRoles(roles), confidence);
    }

    public LimeLightConfig addRole(CameraRole role) {
        EnumSet<CameraRole> newRoles = copyRoles(roles);
        if (role != null) {
            newRoles.add(role);
        }
        return new LimeLightConfig(table, cameraRobotSpace, localizationEstimator, copyTags(filteredTags), useForLocalization, trustDistance, newRoles, confidence);
    }

    public LimeLightConfig setRoles(EnumSet<CameraRole> newRoles) {
        EnumSet<CameraRole> copy = newRoles != null ? copyRoles(newRoles) : EnumSet.noneOf(CameraRole.class);
        return new LimeLightConfig(table, cameraRobotSpace, localizationEstimator, copyTags(filteredTags), useForLocalization, trustDistance, copy, confidence);
    }

    public LimeLightConfig setConfidence(double confidence) {
        return new LimeLightConfig(table, cameraRobotSpace, localizationEstimator, copyTags(filteredTags), useForLocalization, trustDistance, copyRoles(roles), confidence);
    }

    @Override
    public CameraSoftware getSoftware() {
        return CameraSoftware.LimeLight;
    }

    @Override
    public String getTable() {
        return table;
    }

    public LimeLightConfig setCameraRobotSpace(Transform3d transform) {
        return new LimeLightConfig(table, transform, localizationEstimator, copyTags(filteredTags), useForLocalization, trustDistance, copyRoles(roles), confidence);
    }

    public PhotonVisionConfig toSimulationPhotonConfig() {
        PhotonVisionConfig config = PhotonVisionConfig.table(table)
                .setCameraRobotSpace(cameraRobotSpace)
                .setTrustDistance(trustDistance)
                .setUseForLocalization(useForLocalization)
                .setConfidence(confidence)
                .setRoles(copyRoles(roles));

        if (filteredTags != null && filteredTags.length > 0) {
            Integer[] tagArray = Arrays.stream(filteredTags).boxed().toArray(Integer[]::new);
            config = config.setLocalizationTagFilter(tagArray);
        }
        // Approximate Limelight FOV/resolution for simulation
        return config
                .setSimFov(63.3, 49.7)
                .setSimResolution(960, 720)
                .setSimMaxTargets(10);
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
