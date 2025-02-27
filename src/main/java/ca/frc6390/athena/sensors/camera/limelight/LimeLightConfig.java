package ca.frc6390.athena.sensors.camera.limelight;

import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public record LimeLightConfig(String table, Transform3d cameraRobotSpace, PoseEstimateWithLatencyType localizationEstimator) implements ConfigurableCamera {
    private static final String DEFUALT_TABLE = "limelight"; 

    public LimeLightConfig(String table){
        this(table, new Transform3d(), PoseEstimateWithLatencyType.BOT_POSE_MT2_BLUE);
    }

    public LimeLightConfig(){
        this(DEFUALT_TABLE);
    }

    public static LimeLightConfig table(String table){
        return new LimeLightConfig(table);
    }

    public LimeLightConfig setCameraTransform(Transform3d cameraRobotSpace){
        return new LimeLightConfig(table, cameraRobotSpace, localizationEstimator);
    }

    public LimeLightConfig setYawRelativeToForwards(double degrees){
        return new LimeLightConfig(table, new Transform3d(cameraRobotSpace.getTranslation(), new Rotation3d(cameraRobotSpace.getRotation().getX(), cameraRobotSpace.getRotation().getY(), degrees)), localizationEstimator);
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