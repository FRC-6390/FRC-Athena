package ca.frc6390.athena.sensors.camera.photonvision;

import java.util.ArrayList;
import java.util.Arrays;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;

public record PhotonVisionConfig(String table, Transform3d cameraRobotSpace, PoseStrategy poseStrategy, ArrayList<Integer> filteredTags, AprilTagFields fieldLayout, boolean useForLocalization, PoseStrategy poseStrategyFallback) implements ConfigurableCamera {

    @Override
    public Rotation2d getYawRelativeToForwards() {
        return Rotation2d.fromRadians(cameraRobotSpace.getRotation().getZ());
    }

    @Override
    public CameraSoftware getSoftware() {
        return CameraSoftware.PhotonVision;
    }

    public static PhotonVisionConfig table(String table){
        return new PhotonVisionConfig(table, new Transform3d(), PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, new ArrayList<>(), AprilTagFields.k2025ReefscapeWelded, false, PoseStrategy.LOWEST_AMBIGUITY);
    }

    public PhotonVisionConfig setFieldLayout(AprilTagFields fieldLayout){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback);
    }

    public PhotonVisionConfig setCameraRobotSpace(Transform3d cameraRobotSpace){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback);
    }

    public PhotonVisionConfig setCameraRobotSpace(double x){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback);
    }

    public PhotonVisionConfig setTable(String table){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback);
    }

    public PhotonVisionConfig setPoseStrategy(PoseStrategy poseStrategy){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback);
    }

    public PhotonVisionConfig setMultiTagPoseStrategyFallback(PoseStrategy poseStrategyFallback){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy,filteredTags,fieldLayout, useForLocalization, poseStrategyFallback);
    }

    public PhotonVisionConfig setLocalizationTagFilter(Integer... filteredTags){
        var list = new ArrayList<>(Arrays.asList(filteredTags));
        list.add(-1);
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, list,fieldLayout, useForLocalization, poseStrategyFallback);
    }

    public PhotonVisionConfig setUseForLocalization(boolean useForLocalization){
        return new PhotonVisionConfig(table, cameraRobotSpace, poseStrategy, filteredTags,fieldLayout, useForLocalization, poseStrategyFallback);
    }

    @Override
    public String getTable() {
       return table;
    }
    
    public PhotonVision create(){
        return new PhotonVision(table, cameraRobotSpace);
    }
}
