package ca.frc6390.athena.sensors.camera.photonvision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import ca.frc6390.athena.sensors.camera.FiducialSource;
import ca.frc6390.athena.sensors.camera.LocalizationSource;
import ca.frc6390.athena.sensors.camera.TargetingSource;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.sensors.camera.VisionCameraConfig;
import ca.frc6390.athena.sensors.camera.VisionCameraCapability;
import ca.frc6390.athena.sensors.camera.PhotonVisionCamera;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;

public class PhotonVision extends PhotonCamera implements PhotonVisionCamera, LocalizationSource, TargetingSource, FiducialSource {

    private final Transform3d pose;
    private final PhotonVisionConfig config;
    private final PhotonPoseEstimator estimator;
    private final AprilTagFieldLayout fieldLayout;
    private List<PhotonPipelineResult> results;
    private boolean useForLocalization;
    private final VisionCamera localizationCamera;
    private LocalizationSnapshot latestSnapshot = LocalizationSnapshot.empty();
    private double lastTargetYawDegrees = Double.NaN;
    private double lastTargetPitchDegrees = Double.NaN;
    private double lastTargetDistanceMeters = Double.NaN;
    private int lastTargetId = -1;
    private double lastPoseAmbiguity = Double.NaN;
    private double lastCameraPitchDegrees = Double.NaN;
    private double lastCameraRollDegrees = Double.NaN;
    private Pose3d lastEstimatedPose = new Pose3d();
    private List<VisionCamera.TargetMeasurement> latestMeasurements = List.of();

    private static final class LocalizationSnapshot {
        private final Pose2d pose;
        private final Pose3d pose3d;
        private final double latencySeconds;
        private final int visibleTargets;
        private final double totalDistanceMeters;

        private LocalizationSnapshot(
                Pose2d pose, Pose3d pose3d, double latencySeconds, int visibleTargets, double totalDistanceMeters) {
            this.pose = pose;
            this.pose3d = pose3d;
            this.latencySeconds = latencySeconds;
            this.visibleTargets = visibleTargets;
            this.totalDistanceMeters = totalDistanceMeters;
        }

        static LocalizationSnapshot empty() {
            return new LocalizationSnapshot(new Pose2d(), new Pose3d(), 0.0, 0, Double.MAX_VALUE);
        }

        Pose2d pose() {
            return pose;
        }

        Pose3d pose3d() {
            return pose3d;
        }

        double latencySeconds() {
            return latencySeconds;
        }

        int visibleTargets() {
            return visibleTargets;
        }

        double totalDistanceMeters() {
            return totalDistanceMeters;
        }
    }

    private record DistanceObservation(int tagCount, double totalDistanceMeters) {}

    public static PhotonVision fromConfig(PhotonVisionConfig config){
        return new PhotonVision(config);
    }

    public PhotonVision(PhotonVisionConfig config) {
        super(config.table());
        this.config = config;
        this.pose = config.cameraRobotSpace();
        this.fieldLayout = AprilTagFieldLayout.loadField(config.fieldLayout());
        this.estimator = new PhotonPoseEstimator(fieldLayout, pose);
        this.useForLocalization = config.useForLocalization();
        this.localizationCamera = createVisionCamera();
    }

    public PhotonVision(String table, Transform3d pose) {
        this(PhotonVisionConfig.table(table).withCameraRobotSpace(pose));
    }

    public PhotonVisionConfig getConfig() {
        return config;
    }

    private VisionCamera createVisionCamera() {
        VisionCameraConfig cameraConfig =
                VisionCameraConfig.create(config.getTable(), config.getSoftware())
                        .localization(l -> l
                                .useForLocalization(config.useForLocalization())
                                .trustDistance(config.trustDistance())
                                .confidenceSupplier(config::confidence)
                                .roles(config.roles()))
                        .sources(s -> s
                                .connectedSupplier(this::isConnected)
                                .hasTargetsSupplier(this::computeHasTargets)
                                .poseSupplier(this::supplyLocalizationPose)
                                .pose3dSupplier(this::supplyLocalizationPose3d)
                                .latencySupplier(this::supplyLocalizationLatency)
                                .visibleTargetsSupplier(this::supplyVisibleTargets)
                                .averageDistanceSupplier(this::supplyAverageDistance)
                                .orientationConsumer(this::setRobotOrientation)
                                .updateHook(this::refreshLocalizationSnapshot))
                        .transform(t -> t.robotToCameraTransform(config.cameraRobotSpace()))
                        .simulation(s -> s
                                .vfield(v -> v
                                        .horizontalFov(config.simHorizontalFovDeg())
                                        .rangeMeters(Math.max(config.trustDistance(), 2.0))))
                        .targets(t -> t
                                .targetPitchSupplier(this::supplyTargetPitch)
                                .targetYawSupplier(this::supplyTargetYaw)
                                .tagDistanceSupplier(this::supplyTargetDistance)
                                .tagIdSupplier(this::supplyTargetId)
                                .poseAmbiguitySupplier(this::supplyPoseAmbiguity)
                                .cameraPitchSupplier(this::supplyCameraPitch)
                                .cameraRollSupplier(this::supplyCameraRoll)
                                .targetMeasurementsSupplier(this::supplyTargetMeasurements)
                                .fieldLayout(fieldLayout));
        return new VisionCamera(cameraConfig)
                .registerCapability(VisionCameraCapability.PHOTON_VISION_CAMERA, this)
                .registerCapability(VisionCameraCapability.LOCALIZATION_SOURCE, this)
                .registerCapability(VisionCameraCapability.TARGETING_SOURCE, this)
                .registerCapability(VisionCameraCapability.FIDUCIAL_SOURCE, this);
    }

    private void refreshLocalizationSnapshot() {
        updateResults();

        if (!isConnected()) {
            lastTargetYawDegrees = Double.NaN;
            lastTargetPitchDegrees = Double.NaN;
            lastTargetDistanceMeters = Double.NaN;
            lastTargetId = -1;
            lastPoseAmbiguity = Double.NaN;
            lastCameraPitchDegrees = Double.NaN;
            lastCameraRollDegrees = Double.NaN;
            latestSnapshot = LocalizationSnapshot.empty();
            latestMeasurements = List.of();
            clearResults();
            return;
        }

        if (results == null || results.isEmpty()) {
            latestMeasurements = List.of();
            lastPoseAmbiguity = Double.NaN;
            lastCameraPitchDegrees = Double.NaN;
            lastCameraRollDegrees = Double.NaN;
            clearResults();
            return;
        }

        lastTargetYawDegrees = Double.NaN;
        lastTargetPitchDegrees = Double.NaN;
        lastTargetDistanceMeters = Double.NaN;
        lastTargetId = -1;
        lastPoseAmbiguity = Double.NaN;
        lastCameraPitchDegrees = Double.NaN;
        lastCameraRollDegrees = Double.NaN;

        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        List<PhotonTrackedTarget> lastTargets = List.of();
        PhotonTrackedTarget lastBestTarget = null;
        for (PhotonPipelineResult change : results) {
            PhotonPipelineResult filtered = filterResult(change);
            if (filtered == null) {
                continue;
            }
            Optional<EstimatedRobotPose> estimate = estimatePose(filtered);
            List<PhotonTrackedTarget> targets = filtered.getTargets();
            if (estimate.isPresent()) {
                visionEst = estimate;
                lastTargets = targets;
            }
            if (filtered.hasTargets()) {
                var bestTarget = filtered.getBestTarget();
                if (bestTarget != null) {
                    lastBestTarget = bestTarget;
                    lastTargetId = bestTarget.getFiducialId();
                    lastTargetYawDegrees = bestTarget.getYaw();
                    lastTargetPitchDegrees = bestTarget.getPitch();
                    var bestTransform = bestTarget.getBestCameraToTarget();
                    if (bestTransform != null) {
                        lastTargetDistanceMeters = bestTransform.getTranslation().getNorm();
                    }
                    lastPoseAmbiguity = bestTarget.getPoseAmbiguity();
                }
            }
        }

        if (visionEst.isPresent()) {
            EstimatedRobotPose estimate = visionEst.get();
            DistanceObservation stats = calculateDistanceStats(estimate, lastTargets);
            Rotation3d cameraRotation = config.cameraRobotSpace().getRotation();
            lastCameraRollDegrees = Math.toDegrees(cameraRotation.getX());
            lastCameraPitchDegrees = Math.toDegrees(cameraRotation.getY());
            latestSnapshot =
                    new LocalizationSnapshot(
                            estimate.estimatedPose.toPose2d(),
                            estimate.estimatedPose,
                            latencySecondsFromTimestamp(estimate.timestampSeconds),
                            stats.tagCount(),
                            stats.totalDistanceMeters());
        } else {
            latestSnapshot = LocalizationSnapshot.empty();
        }

        latestMeasurements = buildTargetMeasurements(lastBestTarget, lastTargets);
        clearResults();
    }

    private PhotonPipelineResult filterResult(PhotonPipelineResult result) {
        if (result == null || config.filteredTags().isEmpty()) {
            return result;
        }
        List<PhotonTrackedTarget> filteredTargets = result.getTargets().stream()
                .filter(target -> target != null && config.filteredTags().contains(target.getFiducialId()))
                .toList();
        if (filteredTargets.size() == result.getTargets().size()) {
            return result;
        }
        return new PhotonPipelineResult(result.metadata, filteredTargets, Optional.empty());
    }

    private List<VisionCamera.TargetMeasurement> buildTargetMeasurements(
            PhotonTrackedTarget primary, List<PhotonTrackedTarget> targets) {
        if (targets == null || targets.isEmpty()) {
            return List.of();
        }
        List<VisionCamera.TargetMeasurement> measurements = new ArrayList<>();
        if (primary != null) {
            VisionCamera.TargetMeasurement measurement = createMeasurement(primary);
            if (measurement != null) {
                measurements.add(measurement);
            }
        }
        for (PhotonTrackedTarget target : targets) {
            if (target == null || target == primary) {
                continue;
            }
            VisionCamera.TargetMeasurement measurement = createMeasurement(target);
            if (measurement != null) {
                measurements.add(measurement);
            }
        }
        return measurements.isEmpty() ? List.of() : List.copyOf(measurements);
    }

    private VisionCamera.TargetMeasurement createMeasurement(PhotonTrackedTarget target) {
        if (target == null) {
            return null;
        }
        var bestTransform = target.getBestCameraToTarget();
        Translation2d translation2d = null;
        Double distance = null;
        if (bestTransform != null) {
            var translation3d = bestTransform.getTranslation();
            translation2d = new Translation2d(translation3d.getX(), translation3d.getY());
            distance = translation3d.getNorm();
        }
        double yaw = target.getYaw();
        Double yawDegrees = Double.isNaN(yaw) ? null : yaw;
        Double confidence = null;
        double poseAmbiguity = target.getPoseAmbiguity();
        if (!Double.isNaN(poseAmbiguity)) {
            confidence = 1.0 - Math.min(Math.max(poseAmbiguity, 0.0), 1.0);
        }
        return new VisionCamera.TargetMeasurement(
                translation2d,
                yawDegrees,
                distance,
                target.getFiducialId(),
                confidence);
    }

    private DistanceObservation calculateDistanceStats(
            EstimatedRobotPose estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose == null || targets == null) {
            return new DistanceObservation(0, Double.MAX_VALUE);
        }

        int tagCount = 0;
        double totalDistance = 0.0;
        Pose2d estimated = estimatedPose.estimatedPose.toPose2d();

        for (PhotonTrackedTarget target : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(target.getFiducialId());
            if (tagPose.isEmpty()) {
                continue;
            }
            tagCount++;
            totalDistance +=
                    tagPose
                            .get()
                            .toPose2d()
                            .getTranslation()
                            .getDistance(estimated.getTranslation());
        }

        if (tagCount == 0) {
            return new DistanceObservation(0, Double.MAX_VALUE);
        }
        return new DistanceObservation(tagCount, totalDistance);
    }

    private Pose2d supplyLocalizationPose() {
        return latestSnapshot.pose();
    }

    private Pose3d supplyLocalizationPose3d() {
        return latestSnapshot.pose3d();
    }

    private double supplyLocalizationLatency() {
        return latestSnapshot.latencySeconds();
    }

    private double latencySecondsFromTimestamp(double timestampSeconds) {
        if (!Double.isFinite(timestampSeconds)) {
            return Double.NaN;
        }
        double latencySeconds = Timer.getFPGATimestamp() - timestampSeconds;
        if (!Double.isFinite(latencySeconds)) {
            return Double.NaN;
        }
        return Math.max(0.0, latencySeconds);
    }

    private int supplyVisibleTargets() {
        return latestSnapshot.visibleTargets();
    }

    private double supplyAverageDistance() {
        return latestSnapshot.totalDistanceMeters();
    }

    private double supplyTargetPitch() {
        return lastTargetPitchDegrees;
    }

    private double supplyTargetYaw() {
        return lastTargetYawDegrees;
    }

    private double supplyTargetDistance() {
        return lastTargetDistanceMeters;
    }

    private int supplyTargetId() {
        return lastTargetId;
    }

    private double supplyPoseAmbiguity() {
        return lastPoseAmbiguity;
    }

    private double supplyCameraPitch() {
        return lastCameraPitchDegrees;
    }

    private double supplyCameraRoll() {
        return lastCameraRollDegrees;
    }

    private List<VisionCamera.TargetMeasurement> supplyTargetMeasurements() {
        return latestMeasurements;
    }

    private Optional<EstimatedRobotPose> estimatePose(PhotonPipelineResult result) {
        Optional<EstimatedRobotPose> estimate = estimatePose(result, config.poseStrategy());
        estimate.ifPresent(value -> lastEstimatedPose = value.estimatedPose);
        return estimate;
    }

    private Optional<EstimatedRobotPose> estimatePose(
            PhotonPipelineResult result, PhotonPoseEstimator.PoseStrategy strategy) {
        return switch (strategy) {
            case LOWEST_AMBIGUITY -> estimator.estimateLowestAmbiguityPose(result);
            case CLOSEST_TO_CAMERA_HEIGHT -> estimator.estimateClosestToCameraHeightPose(result);
            case CLOSEST_TO_REFERENCE_POSE -> estimator.estimateClosestToReferencePose(result, lastEstimatedPose);
            case CLOSEST_TO_LAST_POSE -> estimator.estimateClosestToReferencePose(result, lastEstimatedPose);
            case AVERAGE_BEST_TARGETS -> estimator.estimateAverageBestTargetsPose(result);
            case MULTI_TAG_PNP_ON_COPROCESSOR -> {
                Optional<EstimatedRobotPose> estimate = estimator.estimateCoprocMultiTagPose(result);
                yield estimate.isPresent() ? estimate : estimatePose(result, config.poseStrategyFallback());
            }
            case MULTI_TAG_PNP_ON_RIO, CONSTRAINED_SOLVEPNP -> estimatePose(result, config.poseStrategyFallback());
            case PNP_DISTANCE_TRIG_SOLVE -> estimator.estimatePnpDistanceTrigSolvePose(result);
        };
    }

    private boolean computeHasTargets() {
        if (!isConnected()) {
            return false;
        }
        updateResults();
        if (results != null && !results.isEmpty()) {
            return results.stream().anyMatch(PhotonPipelineResult::hasTargets);
        }
        return latestSnapshot.visibleTargets() > 0 || !latestMeasurements.isEmpty();
    }

    public VisionCamera getVisionCamera() {
        return localizationCamera;
    }

    public void setRobotOrientation(Pose2d pose){
        estimator.addHeadingData(Timer.getFPGATimestamp(), pose.getRotation());
        lastEstimatedPose = new Pose3d(pose);
    }

    @Override
    public Pose2d getLocalizationPose() {
        return localizationCamera.getLocalizationPose();
    }

    @Override
    public double getLocalizationLatency() {
        return localizationCamera.getLocalizationLatency();
    }

    @Override
    public VisionCamera.LocalizationData getLocalizationData() {
        return localizationCamera.getLocalizationData();
    }

    @Override
    public Optional<VisionCamera.VisionMeasurement> getLatestVisionMeasurement() {
        return localizationCamera.getLatestVisionMeasurement();
    }

    @Override
    public boolean hasValidTarget() {
        return localizationCamera.hasValidTarget();
    }

    @Override
    public OptionalDouble getTargetYawDegrees() {
        return localizationCamera.getTargetYawDegrees();
    }

    @Override
    public OptionalDouble getTargetPitchDegrees() {
        return localizationCamera.getTargetPitchDegrees();
    }

    @Override
    public OptionalDouble getTargetDistanceMeters() {
        return localizationCamera.getTargetDistanceMeters();
    }

    @Override
    public OptionalInt getLatestTagId() {
        return localizationCamera.getLatestTagId();
    }

    @Override
    public List<VisionCamera.TargetMeasurement> getTargetMeasurements() {
        return localizationCamera.getTargetMeasurements();
    }

    @Override
    public Optional<VisionCamera.TargetObservation> getLatestObservation(
            VisionCamera.CoordinateSpace space, Pose2d robotPose, Pose2d tagPose) {
        return localizationCamera.getLatestObservation(space, robotPose, tagPose);
    }

    @Override
    public Optional<Pose2d> estimateFieldPoseFromTag(int tagId) {
        return localizationCamera.estimateFieldPoseFromTag(tagId);
    }

    @Override
    public Optional<Pose2d> estimateFieldPoseFromTag(int tagId, Translation2d cameraOffsetMeters) {
        return localizationCamera.estimateFieldPoseFromTag(tagId, cameraOffsetMeters);
    }

    private void updateResults(){
        if(results == null){
            results = getAllUnreadResults();
        }
    }

    private void clearResults(){
        results = null;
    }

    @Override
    public Matrix<N3, N1> getLocalizationStdDevs() {
        return localizationCamera.getLocalizationStdDevs();
    }

    public PhotonVision stdDevs(Matrix<N3, N1> single, Matrix<N3, N1> multi) {
        localizationCamera.config().stdDevs(single, multi);
        return this;
    }

    public Matrix<N3, N1> getSingleStdDev() {
        return localizationCamera.getSingleStdDev();
    }

    public Matrix<N3, N1> getMultiStdDev() {
        return localizationCamera.getMultiStdDev();
    }
    
    public boolean isUseForLocalization() {
        return localizationCamera.isUseForLocalization();
    }

    public PhotonVision useForLocalization(boolean useForLocalization) {
        this.useForLocalization = useForLocalization;
        localizationCamera.config().useForLocalization(useForLocalization);
        return this;
    }
}
