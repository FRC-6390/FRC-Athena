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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;


public class PhotonVision extends PhotonCamera implements LocalizationCamera{

    private final Transform3d pose;
    private final PhotonVisionConfig config;
    private final PhotonPoseEstimator estimator;
    private List<PhotonPipelineResult> results;
    private double latency = 0;

    public static PhotonVision fromConfig(PhotonVisionConfig config){
        return new PhotonVision(config);
    }

    public PhotonVision(PhotonVisionConfig config) {
        super(config.table());
        this.config = config;
        this.pose = config.cameraRobotSpace();
        this.estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(config.fieldLayout()), config.poseStrategy() , pose);
        estimator.setMultiTagFallbackStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
    }

    public PhotonVision(String table, Transform3d pose) {
        this(PhotonVisionConfig.table(table).setCameraRobotSpace(pose));
    }

    public PhotonVisionConfig getConfig() {
        return config;
    }

    @Override
    public void setRobotOrientation(Double[] orientation){
        estimator.addHeadingData(Timer.getFPGATimestamp(), Rotation2d.fromDegrees(orientation[0]));
    }

    @Override
    public Pose2d getLocalizationPose() {
        updateResults();
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        if(!isConnected()) return null;
        for (var change : results) {       
            visionEst = estimator.update(change);
        }
        
        if (visionEst.isPresent()) {
            latency = visionEst.get().timestampSeconds;
            return visionEst.get().estimatedPose.toPose2d();
        }
        clearResults();
        return null;
    }

    @Override
    public double getLocalizationLatency() {
       return latency;
    }

    @Override
    public boolean hasValidTarget() {
        if(!isConnected()) return false;
        updateResults();
        if(results.size() == 0) return false;
        return results.stream().anyMatch(r -> r.hasTargets());
    }

    private void updateResults(){
        if(results == null){
            results = getAllUnreadResults();
        }
    }

    private void clearResults(){
        results = null;
    }
}
