package ca.frc6390.athena.sensors.camera.photonvision;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public record PhotonVisionConfig(String table, Transform3d cameraRobotSpace, PoseStrategy poseStrategy, double[] ignoreTags, AprilTagFields fieldLayout) implements ConfigurableCamera {

    @Override
    public Rotation2d getYawRelativeToForwards() {
        return Rotation2d.fromRadians(cameraRobotSpace.getRotation().getZ());
    }

    @Override
    public CameraSoftware getSoftware() {
        return CameraSoftware.PhotonVision;
    }

    public static PhotonVisionConfig table(String table){
        return new PhotonVisionConfig(table, new Transform3d(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new double[]{}, AprilTagFields.k2025ReefscapeWelded);
    }

    public PhotonVisionConfig setFieldLayout(AprilTagFields fieldLayout){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,ignoreTags,fieldLayout);
    }

    public PhotonVisionConfig setCameraRobotSpace(Transform3d cameraRobotSpace){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,ignoreTags,fieldLayout);
    }

    public PhotonVisionConfig setCameraRobotSpace(double x){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,ignoreTags,fieldLayout);
    }

    public PhotonVisionConfig setTable(String table){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,ignoreTags,fieldLayout);
    }

    public PhotonVisionConfig setPoseStrategy(PoseStrategy poseStrategy){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,ignoreTags,fieldLayout);
    }

     public PhotonVisionConfig setLocalizationTagExcludeList(double... ignoreTags){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, ignoreTags,fieldLayout);
    }

    @Override
    public String getTable() {
       return table;
    }
    
    public PhotonVision create(){
        return new PhotonVision(table, cameraRobotSpace);
    }
}
