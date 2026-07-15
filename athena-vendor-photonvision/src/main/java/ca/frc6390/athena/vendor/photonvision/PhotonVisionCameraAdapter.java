package ca.frc6390.athena.vendor.photonvision;

import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.Supplier;

import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.api.hardware.CameraKinds;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.measurement.TargetMeasurementSample;
import ca.frc6390.athena.vision.device.CameraDevice;
import ca.frc6390.athena.vision.device.CameraMountPose;
import ca.frc6390.athena.vision.device.PhotonVisionDevice;
import ca.frc6390.athena.vision.signal.PoseSignal;
import ca.frc6390.athena.vision.signal.TargetSignal;
import ca.frc6390.athena.vision.runtime.CameraAdapter;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * PhotonVision camera adapter backed by PhotonLib.
 */
public final class PhotonVisionCameraAdapter implements CameraAdapter, AutoCloseable {
    private final PhotonVisionDevice device;
    private final PhotonClient client;
    private final PhotonPoseClient poseClient;

    /**
     * Creates an adapter for support checks and static target conversion only.
     */
    public PhotonVisionCameraAdapter() {
        device = null;
        client = null;
        poseClient = null;
    }

    /**
     * Creates an adapter using a real PhotonLib camera.
     *
     * @param device PhotonVision camera device
     */
    public PhotonVisionCameraAdapter(PhotonVisionDevice device) {
        this(
                device,
                new PhotonCameraClient(new PhotonCamera(Objects.requireNonNull(device, "device").name())),
                defaultPoseClient(device::mountPose));
    }

    /**
     * Creates an adapter using a real PhotonLib camera and pose estimator.
     *
     * @param device PhotonVision camera device
     * @param poseEstimator PhotonLib pose estimator
     */
    public PhotonVisionCameraAdapter(PhotonVisionDevice device, PhotonPoseEstimator poseEstimator) {
        this(device, poseEstimator, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    }

    /**
     * Creates an adapter using a real PhotonLib camera and pose estimator.
     *
     * @param device PhotonVision camera device
     * @param poseEstimator PhotonLib pose estimator
     * @param poseStrategy pose strategy to use when reading estimates
     */
    public PhotonVisionCameraAdapter(
            PhotonVisionDevice device, PhotonPoseEstimator poseEstimator, PhotonPoseEstimator.PoseStrategy poseStrategy) {
        this(
                device,
                new PhotonCameraClient(new PhotonCamera(Objects.requireNonNull(device, "device").name())),
                new PhotonPoseEstimatorClient(poseEstimator, poseStrategy, device::mountPose));
    }

    PhotonVisionCameraAdapter(PhotonVisionDevice device, PhotonClient client) {
        this(device, client, null);
    }

    PhotonVisionCameraAdapter(PhotonVisionDevice device, PhotonClient client, PhotonPoseClient poseClient) {
        this.device = Objects.requireNonNull(device, "device");
        this.client = Objects.requireNonNull(client, "client");
        this.poseClient = poseClient;
    }

    /**
     * Returns true when a camera kind is handled by this adapter.
     *
     * @param kind camera kind
     * @return true if supported
     */
    public boolean supports(CameraKind kind) {
        return kind == CameraKinds.PHOTONVISION || kind.key().equals(CameraKinds.PHOTONVISION.key());
    }

    /**
     * Returns true when a camera device is handled by this adapter.
     *
     * @param device camera device
     * @return true if supported
     */
    public boolean supports(PhotonVisionDevice device) {
        return device != null && supports(device.kind());
    }

    @Override
    public boolean supports(CameraDevice camera) {
        return camera instanceof PhotonVisionDevice photon && supports(photon);
    }

    @Override
    public CameraDevice bind(CameraDevice camera) {
        if (camera instanceof PhotonVisionDevice photon) {
            return new PhotonVisionCameraAdapter(photon).bind();
        }
        throw new IllegalArgumentException("PhotonVision adapter cannot bind camera " + camera.name());
    }

    /**
     * Returns a target signal backed by this adapter.
     *
     * @return target signal
     */
    public TargetSignal targetSignal() {
        ensureConfigured();
        return device.bindTargets(new PhotonVisionMeasurementBuffer(device, client, poseClient)::targetMeasurements).targets();
    }

    /**
     * Returns a pose signal backed by this adapter.
     *
     * @return pose signal
     */
    public PoseSignal poseSignal() {
        ensureConfigured();
        ensurePoseConfigured();
        return device.bindPose(new PhotonVisionMeasurementBuffer(device, client, poseClient)::poseMeasurements).pose();
    }

    /**
     * Binds this adapter's configured signals to its camera declaration.
     *
     * @return updated device declaration
     */
    public PhotonVisionDevice bind() {
        ensureConfigured();
        PhotonVisionMeasurementBuffer buffer = new PhotonVisionMeasurementBuffer(device, client, poseClient);
        PhotonVisionDevice bound = device.bindTargets(buffer::targetMeasurements);
        return poseClient == null ? bound : bound.bindPose(buffer::poseMeasurements);
    }

    /**
     * Returns latest unread target measurements from the configured camera.
     *
     * @return target measurements
     */
    public List<Measurement> latestTargets() {
        ensureConfigured();
        List<PhotonVisionResult> results = client.unreadResults(null);
        if (results.isEmpty()) {
            return List.of();
        }
        return targetMeasurementsFromResult(results.get(results.size() - 1), device);
    }

    /**
     * Returns latest unread pose measurements from the configured camera.
     *
     * @return pose measurements
     */
    public List<Measurement> latestPoses() {
        ensureConfigured();
        ensurePoseConfigured();
        List<PhotonVisionResult> results = client.unreadResults(poseClient);
        if (results.isEmpty()) {
            return List.of();
        }
        return poseMeasurementsFromResult(results.get(results.size() - 1), device);
    }

    /**
     * Returns the camera device used by this adapter.
     *
     * @return camera device
     */
    public PhotonVisionDevice device() {
        return device;
    }

    static List<Measurement> measurementsFromTargets(List<PhotonVisionTarget> targets, Object source) {
        if (targets == null || targets.isEmpty()) {
            return List.of();
        }
        return targets.stream()
                .filter(Objects::nonNull)
                .map(target -> targetMeasurement(target, 0.0, 0.0, source))
                .map(Measurement.class::cast)
                .toList();
    }

    /**
     * Converts a real PhotonLib pipeline result into measurement samples.
     *
     * @param result PhotonLib pipeline result
     * @param source measurement source
     * @return measurements
     */
    public static List<Measurement> measurementsFromResult(PhotonPipelineResult result, Object source) {
        return targetMeasurementsFromResult(PhotonVisionResult.fromPhoton(result, null), source);
    }

    static List<Measurement> targetMeasurementsFromResult(PhotonVisionResult result, Object source) {
        if (result == null || !result.hasTargets()) {
            return List.of();
        }
        return result.targets().stream()
                .map(target -> targetMeasurement(target, result.timestampSeconds(), result.latencySeconds(), source))
                .map(Measurement.class::cast)
                .toList();
    }

    static List<Measurement> poseMeasurementsFromResult(PhotonVisionResult result, Object source) {
        if (result == null || result.pose().isEmpty()) {
            return List.of();
        }
        PhotonVisionPoseEstimate pose = result.pose().get();
        return List.of(new PhotonVisionPoseMeasurement(
                pose.pose(),
                RobotVelocity.zero(),
                pose.timestampSeconds(),
                result.latencySeconds(),
                pose.ambiguity(),
                pose.targetCount(),
                averageTargetDistance(result.targets()),
                MeasurementStdDevs.of(Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY, Double.POSITIVE_INFINITY),
                source));
    }

    private static double averageTargetDistance(List<PhotonVisionTarget> targets) {
        return targets == null || targets.isEmpty()
                ? Double.NaN
                : targets.stream()
                        .mapToDouble(PhotonVisionTarget::distanceMeters)
                        .filter(Double::isFinite)
                        .average()
                        .orElse(Double.NaN);
    }

    /**
     * Converts a real PhotonLib tracked target into a vendor target value.
     *
     * @param target PhotonLib target
     * @return target value
     */
    static PhotonVisionTarget targetFromPhoton(PhotonTrackedTarget target) {
        Objects.requireNonNull(target, "target");
        var transform = target.getBestCameraToTarget();
        var translation = transform.getTranslation();
        return new PhotonVisionTarget(
                target.getFiducialId(),
                target.getYaw(),
                target.getPitch(),
                translation.getNorm(),
                target.getPoseAmbiguity());
    }

    private void ensureConfigured() {
        if (client == null) {
            throw new IllegalStateException("PhotonVision camera adapter was created without a camera device.");
        }
    }

    private void ensurePoseConfigured() {
        if (poseClient == null) {
            throw new IllegalStateException("PhotonVision pose estimation requires a PhotonPoseEstimator.");
        }
    }

    private static double latencySeconds(PhotonPipelineResult result) {
        if (result == null || result.metadata == null) {
            return 0.0;
        }
        return Math.max(0.0, finiteOrZero(result.metadata.getLatencyMillis()) / 1000.0);
    }

    private static double finiteOrZero(double value) {
        return Double.isFinite(value) ? value : 0.0;
    }

    private static PhotonVisionTargetMeasurement targetMeasurement(
            PhotonVisionTarget target,
            double timestampSeconds,
            double latencySeconds,
            Object source) {
        return new PhotonVisionTargetMeasurement(
                target.fiducialId(),
                target.yawDegrees(),
                target.pitchDegrees(),
                target.distanceMeters(),
                target.poseAmbiguity(),
                target.confidence(),
                timestampSeconds,
                latencySeconds,
                source);
    }

    @Override
    public void close() {
        if (client != null) {
            client.close();
        }
    }

    interface PhotonClient extends AutoCloseable {
        List<PhotonVisionResult> unreadResults(PhotonPoseClient poseClient);

        @Override
        void close();
    }

    interface PhotonPoseClient {
        Optional<PhotonVisionPoseEstimate> estimate(PhotonPipelineResult result);
    }

    private static final class PhotonCameraClient implements PhotonClient {
        private final PhotonCamera camera;

        private PhotonCameraClient(PhotonCamera camera) {
            this.camera = Objects.requireNonNull(camera, "camera");
        }

        @Override
        public List<PhotonVisionResult> unreadResults(PhotonPoseClient poseClient) {
            return camera.getAllUnreadResults().stream()
                    .map(result -> PhotonVisionResult.fromPhoton(result, poseClient))
                    .toList();
        }

        @Override
        public void close() {
            camera.close();
        }
    }

    static final class PhotonPoseEstimatorClient implements PhotonPoseClient {
        private final PhotonPoseEstimator estimator;
        private final PhotonPoseEstimator.PoseStrategy strategy;
        private final Supplier<CameraMountPose> mountPose;

        private PhotonPoseEstimatorClient(PhotonPoseEstimator estimator, PhotonPoseEstimator.PoseStrategy strategy) {
            this(estimator, strategy, null);
        }

        private PhotonPoseEstimatorClient(
                PhotonPoseEstimator estimator,
                PhotonPoseEstimator.PoseStrategy strategy,
                Supplier<CameraMountPose> mountPose) {
            this.estimator = Objects.requireNonNull(estimator, "estimator");
            this.strategy = strategy == null
                    ? PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                    : strategy;
            this.mountPose = mountPose;
        }

        @Override
        public Optional<PhotonVisionPoseEstimate> estimate(PhotonPipelineResult result) {
            if (mountPose != null) {
                estimator.setRobotToCameraTransform(transform(mountPose.get()));
            }
            return estimateWithConfiguredStrategy(result).map(estimate -> {
                Pose3d pose3d = estimate.estimatedPose;
                var pose2d = pose3d.toPose2d();
                return new PhotonVisionPoseEstimate(
                        new PoseSnapshot(pose2d.getX(), pose2d.getY(), pose2d.getRotation().getRadians()),
                        estimate.timestampSeconds,
                        ambiguity(estimate.targetsUsed),
                        estimate.targetsUsed == null ? 0 : estimate.targetsUsed.size());
            });
        }

        private Optional<org.photonvision.EstimatedRobotPose> estimateWithConfiguredStrategy(
                PhotonPipelineResult result) {
            return switch (strategy) {
                case LOWEST_AMBIGUITY -> estimator.estimateLowestAmbiguityPose(result);
                case CLOSEST_TO_CAMERA_HEIGHT -> estimator.estimateClosestToCameraHeightPose(result);
                case CLOSEST_TO_REFERENCE_POSE -> estimator.estimateClosestToReferencePose(
                        result, Pose3d.kZero);
                case AVERAGE_BEST_TARGETS -> estimator.estimateAverageBestTargetsPose(result);
                case MULTI_TAG_PNP_ON_COPROCESSOR -> multiTagThenSingleTag(
                        estimator.estimateCoprocMultiTagPose(result),
                        () -> estimator.estimateLowestAmbiguityPose(result));
                case PNP_DISTANCE_TRIG_SOLVE -> estimator.estimatePnpDistanceTrigSolvePose(result);
                default -> estimator.estimateCoprocMultiTagPose(result)
                        .or(() -> estimator.estimateLowestAmbiguityPose(result));
            };
        }

        private static double ambiguity(List<PhotonTrackedTarget> targets) {
            if (targets == null || targets.isEmpty()) {
                return 0.0;
            }
            return targets.stream()
                    .filter(Objects::nonNull)
                    .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
                    .filter(Double::isFinite)
                    .min()
                    .orElse(0.0);
        }
    }

    static <T> Optional<T> multiTagThenSingleTag(Optional<T> multiTag, Supplier<Optional<T>> singleTag) {
        Optional<T> primary = multiTag == null ? Optional.empty() : multiTag;
        return primary.isPresent() ? primary : Objects.requireNonNull(singleTag, "singleTag").get();
    }

    private static PhotonPoseClient defaultPoseClient(Supplier<CameraMountPose> mount) {
        Supplier<CameraMountPose> safeMount = mount == null ? CameraMountPose::identity : mount;
        Transform3d robotToCamera = transform(safeMount.get());
        PhotonPoseEstimator estimator = new PhotonPoseEstimator(
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField),
                robotToCamera);
        return new PhotonPoseEstimatorClient(
                estimator,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                safeMount);
    }

    private static Transform3d transform(CameraMountPose mount) {
        CameraMountPose safe = mount == null || !mount.isFinite() ? CameraMountPose.identity() : mount;
        return new Transform3d(
                new Translation3d(safe.xMeters(), safe.yMeters(), safe.zMeters()),
                new Rotation3d(
                        Math.toRadians(safe.rollDegrees()),
                        Math.toRadians(safe.pitchDegrees()),
                        Math.toRadians(safe.yawDegrees())));
    }

    static record PhotonVisionResult(
            double timestampSeconds,
            double latencySeconds,
            List<PhotonVisionTarget> targets,
            Optional<PhotonVisionPoseEstimate> pose) {
        PhotonVisionResult {
            timestampSeconds = finiteOrZero(timestampSeconds);
            latencySeconds = Math.max(0.0, finiteOrZero(latencySeconds));
            targets = targets == null ? List.of() : List.copyOf(targets);
            pose = pose == null ? Optional.empty() : pose;
        }

        boolean hasTargets() {
            return !targets.isEmpty();
        }

        static PhotonVisionResult fromPhoton(PhotonPipelineResult result, PhotonPoseClient poseClient) {
            if (result == null) {
                return new PhotonVisionResult(0.0, 0.0, List.of(), Optional.empty());
            }
            Optional<PhotonVisionPoseEstimate> pose = poseClient == null ? Optional.empty() : poseClient.estimate(result);
            List<PhotonVisionTarget> targets = result.hasTargets()
                    ? result.getTargets().stream().map(PhotonVisionCameraAdapter::targetFromPhoton).toList()
                    : List.of();
            return new PhotonVisionResult(
                    result.getTimestampSeconds(), PhotonVisionCameraAdapter.latencySeconds(result), targets, pose);
        }
    }

    static record PhotonVisionPoseEstimate(
            PoseSnapshot pose,
            double timestampSeconds,
            double ambiguity,
            int targetCount) {
        PhotonVisionPoseEstimate {
            pose = Objects.requireNonNull(pose, "pose");
            timestampSeconds = finiteOrZero(timestampSeconds);
            ambiguity = Math.max(0.0, finiteOrZero(ambiguity));
            targetCount = Math.max(0, targetCount);
        }
    }

    private static final class PhotonVisionMeasurementBuffer {
        private final Object source;
        private final PhotonClient client;
        private final PhotonPoseClient poseClient;
        private PhotonVisionResult latest = new PhotonVisionResult(0.0, 0.0, List.of(), Optional.empty());

        private PhotonVisionMeasurementBuffer(Object source, PhotonClient client, PhotonPoseClient poseClient) {
            this.source = source;
            this.client = Objects.requireNonNull(client, "client");
            this.poseClient = poseClient;
        }

        private List<Measurement> targetMeasurements() {
            refresh();
            return targetMeasurementsFromResult(latest, source);
        }

        private List<Measurement> poseMeasurements() {
            refresh();
            return poseMeasurementsFromResult(latest, source);
        }

        private void refresh() {
            List<PhotonVisionResult> results = client.unreadResults(poseClient);
            if (!results.isEmpty()) {
                latest = results.get(results.size() - 1);
            }
        }
    }

    private record PhotonVisionPoseMeasurement(
            PoseSnapshot pose,
            RobotVelocity speeds,
            double timestampSeconds,
            double latencySeconds,
            double ambiguity,
            int targetCount,
            double averageTargetDistanceMeters,
            MeasurementStdDevs stdDevs,
            Object source) implements PoseMeasurementSample {
    }

    private record PhotonVisionTargetMeasurement(
            int targetId,
            double yawDegrees,
            double pitchDegrees,
            double distanceMeters,
            double poseAmbiguity,
            double confidence,
            double timestampSeconds,
            double latencySeconds,
            Object source)
            implements TargetMeasurementSample {
    }
}
