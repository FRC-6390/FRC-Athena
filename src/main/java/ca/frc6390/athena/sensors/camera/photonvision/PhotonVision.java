package ca.frc6390.athena.sensors.camera.photonvision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import ca.frc6390.athena.sensors.camera.LocalizationCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


public class PhotonVision extends PhotonCamera implements LocalizationCamera{

    private final Transform3d pose;
    private final PhotonVisionConfig config;
    private final PhotonPoseEstimator estimator;

    public static PhotonVision fromConfig(PhotonVisionConfig config){
        return new PhotonVision(config);
    }

    public PhotonVision(PhotonVisionConfig config) {
        super(config.table());
        this.config = config;
        this.pose = config.cameraRobotSpace();
        this.estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded), config.poseStrategy() , pose);
    }

    public PhotonVision(String table, Transform3d pose) {
        this(new PhotonVisionConfig(table, pose,PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR ));
    }

    public PhotonVisionConfig getConfig() {
        return config;
    }

    @Override
    public Pose2d getLocalizationPose() {
         Optional<EstimatedRobotPose> visionEst = Optional.empty();
         if(!isConnected()) return null;
        for (var change : getAllUnreadResults()) {            
            visionEst = estimator.update(change);
        }

        if (visionEst.isPresent()) {
            return visionEst.get().estimatedPose.toPose2d();
        }

        return null;
    }

    @Override
    public double getLocalizationLatency() {
        for (var change : getAllUnreadResults()) {
            return change.getTimestampSeconds();
        }
        return 0;
    }

    @Override
    public boolean hasValidTarget() {
        if(!isConnected()) return false;
        List<PhotonPipelineResult> results = getAllUnreadResults();
        if(results.size() == 0) return false;
        return results.stream().anyMatch(r -> r.hasTargets());
    }
    
}
