package ca.frc6390.athena.sensors.camera.limelight;


import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public record LimeLightConfig(String table, Transform3d cameraRobotSpace, PoseEstimateWithLatencyType localizationEstimator, int[] filteredTags, boolean useForLocalization, double trustDistance) implements ConfigurableCamera {
    private static final String DEFUALT_TABLE = "limelight"; 

    public LimeLightConfig(String table){
        this(table, new Transform3d(), PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE, new int[]{}, false, 2);
    }

    public LimeLightConfig(){
        this(DEFUALT_TABLE);
    }

    public static LimeLightConfig table(String table){
        return new LimeLightConfig(table);
    }

    public LimeLightConfig setCameraTransform(Transform3d cameraRobotSpace){
        return new LimeLightConfig(table, cameraRobotSpace, localizationEstimator, filteredTags, useForLocalization, trustDistance);
    }

    public LimeLightConfig setYawRelativeToForwards(double degrees){
        return new LimeLightConfig(table, new Transform3d(cameraRobotSpace.getTranslation(), new Rotation3d(cameraRobotSpace.getRotation().getX(), cameraRobotSpace.getRotation().getY(), degrees)), localizationEstimator, filteredTags, useForLocalization, trustDistance);
    }

    public LimeLightConfig setPoseEstimateType(PoseEstimateWithLatencyType localizationEstimator){
        return new LimeLightConfig(table, cameraRobotSpace, localizationEstimator, filteredTags, useForLocalization, trustDistance);
    }

    public LimeLightConfig setLocalizationTagFilter(int... filteredTags){
        return new LimeLightConfig(table, cameraRobotSpace, localizationEstimator, filteredTags, useForLocalization, trustDistance);
    }
    
    public LimeLightConfig setUseForLocalization(boolean useForLocalization){
        return new LimeLightConfig(table, cameraRobotSpace, localizationEstimator, filteredTags, useForLocalization, trustDistance);
    }

    public LimeLightConfig setTrustDistance(double trustDistance){
        return new LimeLightConfig(table, cameraRobotSpace, localizationEstimator, filteredTags, useForLocalization, trustDistance);
    }

    @Override
    public Rotation2d getYawRelativeToForwards() {
       return Rotation2d.fromRadians(cameraRobotSpace.getRotation().getZ());
    }

    @Override
    public CameraSoftware getSoftware() {
        return CameraSoftware.LimeLight;
    }

    @Override
    public String getTable() {
        return table;
    }
}