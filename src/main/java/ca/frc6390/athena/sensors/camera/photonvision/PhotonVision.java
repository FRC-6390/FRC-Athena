package ca.frc6390.athena.sensors.camera.photonvision;

import java.util.List;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import ca.frc6390.athena.sensors.camera.LocalizationCamera;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.apriltag.AprilTagFieldLayout;


public class PhotonVision extends PhotonCamera implements LocalizationCamera{

    private final Transform3d pose;
    private final PhotonVisionConfig config;
    private final PhotonPoseEstimator estimator;
    private List<PhotonPipelineResult> results;
    private double latency = 0;
    private Matrix<N3, N1> curStdDevs, singleTagStdDevs, multiTagStdDevs;
    public boolean useForLocalization = false;

    public static PhotonVision fromConfig(PhotonVisionConfig config){
        return new PhotonVision(config);
    }

    public PhotonVision(PhotonVisionConfig config) {
        super(config.table());
        this.config = config;
        this.pose = config.cameraRobotSpace();
        this.estimator = new PhotonPoseEstimator(AprilTagFieldLayout.loadField(config.fieldLayout()), config.poseStrategy() , pose);
        estimator.setMultiTagFallbackStrategy(config.poseStrategyFallback());
        this.useForLocalization = config.useForLocalization();
    }

    public PhotonVision(String table, Transform3d pose) {
        this(PhotonVisionConfig.table(table).setCameraRobotSpace(pose));
    }

    public PhotonVisionConfig getConfig() {
        return config;
    }

    @Override
    public void setRobotOrientation(Pose2d pose){
        estimator.addHeadingData(Timer.getFPGATimestamp(), pose.getRotation());
        estimator.setLastPose(pose);
    }

    @Override
    public Pose2d getLocalizationPose() {
        updateResults();
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        if(!isConnected()) {
            clearResults();
            return null;
        }

        for (var change : results) {
            // List<PhotonTrackedTarget> filtered = getConfig().filteredTags().size() > 0 ? change.getTargets().stream().filter(t -> getConfig().filteredTags().contains(t.getFiducialId())).collect(Collectors.toList()) : change.getTargets();
            visionEst = estimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
        }
        
        if (visionEst.isPresent()) {
            latency = visionEst.get().timestampSeconds;
            clearResults();
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

    private void updateEstimationStdDevs(Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = getSingleStdDev();

        } else {
            // Pose present. Start running Heuristic
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            curStdDevs = recalculateStdDevs(numTags, avgDist, config.trustDistance());
        }
    }

    @Override
    public Matrix<N3, N1> getLocalizationStdDevs() {
        return curStdDevs;
    }

    @Override
    public void setStdDevs(Matrix<N3, N1> single, Matrix<N3, N1> multi) {
        singleTagStdDevs = single;
        multiTagStdDevs = multi;
    }

    @Override
    public Matrix<N3, N1> getSingleStdDev() {
        return singleTagStdDevs;
    }

    @Override
    public Matrix<N3, N1> getMultiStdDev() {
        return multiTagStdDevs;
    }
    
    public boolean isUseForLocalization() {
        return useForLocalization;
    }

    public void setUseForLocalization(boolean useForLocalization) {
        this.useForLocalization = useForLocalization;
    }
}
