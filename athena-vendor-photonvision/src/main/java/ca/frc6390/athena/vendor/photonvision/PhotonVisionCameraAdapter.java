package ca.frc6390.athena.vendor.photonvision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import ca.frc6390.athena.api.hardware.CameraKind;
import ca.frc6390.athena.api.hardware.CameraKinds;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.measurement.TargetMeasurementSample;
import ca.frc6390.athena.vision.device.CameraClock;
import ca.frc6390.athena.vision.device.CameraClockStatus;
import ca.frc6390.athena.vision.device.CameraDevice;
import ca.frc6390.athena.vision.device.CameraMountPose;
import ca.frc6390.athena.vision.device.PhotonVisionDevice;
import ca.frc6390.athena.vision.device.PhotonVisionPoseStrategy;
import ca.frc6390.athena.vision.signal.PoseSignal;
import ca.frc6390.athena.vision.signal.TargetSignal;
import ca.frc6390.athena.vision.runtime.CameraAdapter;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/**
 * PhotonVision camera adapter backed by PhotonLib.
 */
public final class PhotonVisionCameraAdapter implements CameraAdapter, AutoCloseable, CameraClock {
    private static volatile AprilTagFieldLayout defaultFieldLayout;
    private static volatile Supplier<Rotation2d> robotHeadingSource;

    /** Robot time source that reports "no clock" (used by adapters built without a robot clock). */
    private static final DoubleSupplier NO_ROBOT_CLOCK = () -> Double.NaN;

    private final PhotonVisionDevice device;
    private final PhotonClient client;
    private final PhotonPoseClient poseClient;
    private final DoubleSupplier robotClock;
    private final ClockMonitor clockMonitor = new ClockMonitor();

    /**
     * Creates an adapter for support checks and static target conversion only.
     */
    public PhotonVisionCameraAdapter() {
        device = null;
        client = null;
        poseClient = null;
        robotClock = NO_ROBOT_CLOCK;
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
                defaultPoseClient(device, defaultFieldLayout()),
                Timer::getFPGATimestamp);
    }

    /**
     * Creates an adapter using a real PhotonLib camera and an explicit AprilTag field layout.
     *
     * <p>This is a factory rather than a constructor on purpose: {@link java.util.ServiceLoader}
     * reflects over every public constructor, so a constructor mentioning a WPILib AprilTag type
     * would stop adapter discovery on classpaths without apriltag-java.</p>
     *
     * @param device PhotonVision camera device
     * @param fieldLayout AprilTag layout used for pose estimation; null uses {@link #defaultFieldLayout()}
     * @return adapter
     */
    public static PhotonVisionCameraAdapter withFieldLayout(
            PhotonVisionDevice device, AprilTagFieldLayout fieldLayout) {
        return new PhotonVisionCameraAdapter(
                device,
                new PhotonCameraClient(new PhotonCamera(Objects.requireNonNull(device, "device").name())),
                defaultPoseClient(device, fieldLayout == null ? defaultFieldLayout() : fieldLayout),
                Timer::getFPGATimestamp);
    }

    /**
     * Sets the robot heading used by heading-constrained single-tag solves when no reference pose is supplied.
     *
     * <p>Call this in the robot constructor or when the gyro is configured, for example
     * {@code PhotonVisionCameraAdapter.defaultRobotHeading(() -> gyroRotation)}. Passing null disables
     * the heading source, and single-tag frames then use the lowest-ambiguity solve.</p>
     *
     * @param heading robot heading supplier, or null
     */
    public static void defaultRobotHeading(Supplier<Rotation2d> heading) {
        robotHeadingSource = heading;
    }

    /**
     * Returns the robot heading source used by heading-constrained single-tag solves.
     *
     * @return heading supplier, or null when none is configured
     */
    public static Supplier<Rotation2d> defaultRobotHeading() {
        return robotHeadingSource;
    }

    /**
     * Sets the AprilTag field layout used by cameras that the runtime binds automatically.
     *
     * <p>Call this before the robot runtime starts (for example in the robot constructor) to use
     * a non-default field, such as a practice-field or district layout. Passing null restores
     * WPILib's {@link AprilTagFields#kDefaultField}.</p>
     *
     * @param fieldLayout field layout, or null for the WPILib default field
     */
    public static void defaultFieldLayout(AprilTagFieldLayout fieldLayout) {
        defaultFieldLayout = fieldLayout;
    }

    /**
     * Returns the AprilTag field layout used by automatically bound cameras.
     *
     * @return field layout
     */
    public static AprilTagFieldLayout defaultFieldLayout() {
        AprilTagFieldLayout layout = defaultFieldLayout;
        if (layout == null) {
            synchronized (PhotonVisionCameraAdapter.class) {
                if (defaultFieldLayout == null) {
                    defaultFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
                }
                layout = defaultFieldLayout;
            }
        }
        return layout;
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
        this(device, poseEstimator, poseStrategy, null);
    }

    /**
     * Creates an adapter using a real PhotonLib camera, pose estimator, and reference pose.
     *
     * <p>The reference pose is used by {@code CLOSEST_TO_REFERENCE_POSE}; supply the current robot
     * estimate (for example {@code localization::pose2d}). When the supplier is null or returns
     * null, that strategy falls back to the lowest-ambiguity single-tag solution.</p>
     *
     * @param device PhotonVision camera device
     * @param poseEstimator PhotonLib pose estimator
     * @param poseStrategy pose strategy to use when reading estimates
     * @param referencePose current robot pose estimate, or null
     */
    public PhotonVisionCameraAdapter(
            PhotonVisionDevice device,
            PhotonPoseEstimator poseEstimator,
            PhotonPoseEstimator.PoseStrategy poseStrategy,
            Supplier<Pose2d> referencePose) {
        this(
                device,
                new PhotonCameraClient(new PhotonCamera(Objects.requireNonNull(device, "device").name())),
                new PhotonPoseEstimatorClient(
                        poseEstimator, poseStrategy, device::mountPose, referencePose, device::singleTagHeadingEnabled,
                        () -> device.reportedRobotHeading().in(Radians)),
                Timer::getFPGATimestamp);
    }

    PhotonVisionCameraAdapter(PhotonVisionDevice device, PhotonClient client) {
        this(device, client, null);
    }

    PhotonVisionCameraAdapter(PhotonVisionDevice device, PhotonClient client, PhotonPoseClient poseClient) {
        this(device, client, poseClient, NO_ROBOT_CLOCK);
    }

    PhotonVisionCameraAdapter(
            PhotonVisionDevice device, PhotonClient client, PhotonPoseClient poseClient, DoubleSupplier robotClock) {
        this.device = Objects.requireNonNull(device, "device");
        this.client = Objects.requireNonNull(client, "client");
        this.poseClient = poseClient;
        this.robotClock = Objects.requireNonNull(robotClock, "robotClock");
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
        return device.bindTargets(bindBuffer()::targetMeasurements).targets();
    }

    /**
     * Returns a pose signal backed by this adapter.
     *
     * @return pose signal
     */
    public PoseSignal poseSignal() {
        ensureConfigured();
        ensurePoseConfigured();
        return device.bindPose(bindBuffer()::poseMeasurements).pose();
    }

    /**
     * Binds this adapter's configured signals to its camera declaration.
     *
     * @return updated device declaration
     */
    public PhotonVisionDevice bind() {
        ensureConfigured();
        PhotonVisionMeasurementBuffer buffer = bindBuffer();
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

    /**
     * Returns the camera clock status as of the most recent refresh.
     *
     * <p>A status is only computed once the adapter has been refreshed with a robot clock (after
     * {@link #bind()} or a signal read in robot code). Until then, and always for adapters built
     * without a clock, this returns {@link CameraClockStatus#unknown()}. {@code synced} is true when the
     * last PhotonVision time-sync confirmation is younger than one second and the latest frame is not
     * stamped more than 0.1 s ahead of robot time.</p>
     *
     * @return clock status; never null
     */
    @Override
    public CameraClockStatus clockStatus() {
        return clockMonitor.status();
    }

    private PhotonVisionMeasurementBuffer bindBuffer() {
        ensureConfigured();
        device.bindClockStatus(clockMonitor::status);
        return new PhotonVisionMeasurementBuffer(device, client, poseClient, clockMonitor, robotClock);
    }

    /**
     * Computes the clock status for the latest frame.
     *
     * @param result latest frame read from the camera, or null
     * @param receivedAtSeconds robot time when that frame was read; NaN if never read
     * @param refreshedAtSeconds robot time of the latest refresh; NaN if no robot clock
     * @return clock status
     */
    static CameraClockStatus clockStatus(
            PhotonVisionResult result, double receivedAtSeconds, double refreshedAtSeconds) {
        if (result == null || !Double.isFinite(receivedAtSeconds) || !Double.isFinite(refreshedAtSeconds)) {
            return CameraClockStatus.unknown();
        }
        // The pong age was true when the frame was read; add the time since then.
        double pongAgeSeconds = result.secondsSinceLastPong()
                + Math.max(0.0, refreshedAtSeconds - receivedAtSeconds);
        return CameraClockStatus.of(pongAgeSeconds, result.timestampSeconds(), refreshedAtSeconds);
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
                Double.isFinite(pose.averageTargetDistanceMeters())
                        ? pose.averageTargetDistanceMeters()
                        : averageTargetDistance(result.targets()),
                MeasurementStdDevs.of(Meters.of(Double.POSITIVE_INFINITY), Meters.of(Double.POSITIVE_INFINITY), Radians.of(Double.POSITIVE_INFINITY)),
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
        default boolean isConnected() { return true; }

        List<PhotonVisionResult> unreadResults(PhotonPoseClient poseClient);

        @Override
        void close();
    }

    interface PhotonPoseClient {
        Optional<PhotonVisionPoseEstimate> estimate(PhotonPipelineResult result);

        /**
         * Called once per robot cycle before frames are read, with the robot time of that cycle.
         *
         * @param robotTimeSeconds robot time of the cycle; NaN when no robot clock is available
         */
        default void beginCycle(double robotTimeSeconds) {
        }
    }

    private static final class PhotonCameraClient implements PhotonClient {
        private final PhotonCamera camera;

        private PhotonCameraClient(PhotonCamera camera) {
            this.camera = Objects.requireNonNull(camera, "camera");
        }

        @Override
        public List<PhotonVisionResult> unreadResults(PhotonPoseClient poseClient) {
            List<PhotonPipelineResult> results = camera.getAllUnreadResults();
            if (results.isEmpty()) return List.of();
            return List.of(PhotonVisionResult.fromPhoton(results.get(results.size() - 1), poseClient));
        }

        @Override
        public boolean isConnected() {
            return camera.isConnected();
        }

        @Override
        public void close() {
            camera.close();
        }
    }

    static final class PhotonPoseEstimatorClient implements PhotonPoseClient {
        private static final double SAME_POSE_TOLERANCE_METERS = 1.0e-6;
        private final PhotonPoseEstimator estimator;
        private final PhotonPoseEstimator.PoseStrategy strategy;
        private final Supplier<CameraMountPose> mountPose;
        private final Supplier<Pose2d> referencePose;
        /** Heading the localization reports to the camera declaration every cycle (NaN before the first). */
        private final java.util.function.DoubleSupplier reportedHeadingRadians;
        private final BooleanSupplier singleTagUsesHeading;

        private PhotonPoseEstimatorClient(
                PhotonPoseEstimator estimator,
                PhotonPoseEstimator.PoseStrategy strategy,
                Supplier<CameraMountPose> mountPose) {
            this(estimator, strategy, mountPose, null);
        }

        PhotonPoseEstimatorClient(
                PhotonPoseEstimator estimator,
                PhotonPoseEstimator.PoseStrategy strategy,
                Supplier<CameraMountPose> mountPose,
                Supplier<Pose2d> referencePose) {
            this(estimator, strategy, mountPose, referencePose, () -> false);
        }

        PhotonPoseEstimatorClient(
                PhotonPoseEstimator estimator,
                PhotonPoseEstimator.PoseStrategy strategy,
                Supplier<CameraMountPose> mountPose,
                Supplier<Pose2d> referencePose,
                BooleanSupplier singleTagUsesHeading) {
            this(estimator, strategy, mountPose, referencePose, singleTagUsesHeading, () -> Double.NaN);
        }

        PhotonPoseEstimatorClient(
                PhotonPoseEstimator estimator,
                PhotonPoseEstimator.PoseStrategy strategy,
                Supplier<CameraMountPose> mountPose,
                Supplier<Pose2d> referencePose,
                BooleanSupplier singleTagUsesHeading,
                java.util.function.DoubleSupplier reportedHeadingRadians) {
            this.reportedHeadingRadians = reportedHeadingRadians == null ? () -> Double.NaN : reportedHeadingRadians;
            this.estimator = Objects.requireNonNull(estimator, "estimator");
            this.strategy = strategy == null
                    ? PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                    : strategy;
            this.mountPose = mountPose;
            this.referencePose = referencePose;
            this.singleTagUsesHeading = singleTagUsesHeading == null ? () -> false : singleTagUsesHeading;
        }

        /**
         * Feeds the robot heading into the estimator's heading buffer, timestamped with this cycle's robot time.
         * PNP_DISTANCE_TRIG_SOLVE reads that buffer at each frame's capture time.
         */
        @Override
        public void beginCycle(double robotTimeSeconds) {
            if (!Double.isFinite(robotTimeSeconds) || !singleTagUsesHeading.getAsBoolean()) {
                return;
            }
            Rotation2d heading = robotHeading();
            if (heading != null) {
                estimator.addHeadingData(robotTimeSeconds, heading);
            }
        }

        private Rotation2d robotHeading() {
            if (referencePose != null) {
                Pose2d reference = referencePose.get();
                if (reference != null) {
                    return reference.getRotation();
                }
            }
            double reported = reportedHeadingRadians.getAsDouble();
            if (Double.isFinite(reported)) {
                return new Rotation2d(reported);
            }
            Supplier<Rotation2d> source = robotHeadingSource;
            return source == null ? null : source.get();
        }

        @Override
        public Optional<PhotonVisionPoseEstimate> estimate(PhotonPipelineResult result) {
            if (mountPose != null) {
                estimator.setRobotToCameraTransform(transform(mountPose.get()));
            }
            return estimateWithConfiguredStrategy(result).map(estimate -> toEstimate(
                    estimate, estimator.getFieldTags(), estimator.getRobotToCameraTransform()));
        }

        static PhotonVisionPoseEstimate toEstimate(
                org.photonvision.EstimatedRobotPose estimate,
                AprilTagFieldLayout layout,
                Transform3d robotToCamera) {
            var pose2d = estimate.estimatedPose.toPose2d();
            PoseSnapshot pose = new PoseSnapshot(pose2d.getX(), pose2d.getY(), pose2d.getRotation().getRadians());
            List<PhotonTrackedTarget> used = estimate.targetsUsed == null ? List.of() : estimate.targetsUsed;
            if (isMultiTag(estimate.strategy)) {
                return new PhotonVisionPoseEstimate(
                        pose, estimate.timestampSeconds, ambiguity(used), used.size());
            }
            if (estimate.strategy == PhotonPoseEstimator.PoseStrategy.AVERAGE_BEST_TARGETS) {
                // An average of single-tag solves is still ambiguous; report the worst tag so
                // ambiguity filters stay meaningful.
                return new PhotonVisionPoseEstimate(
                        pose, estimate.timestampSeconds, maxAmbiguity(used), used.size());
            }
            // PhotonLib reports every visible target in targetsUsed even for single-tag solves.
            // Report the one tag that produced the pose so single-tag covariance and
            // VisionFilters.maxAmbiguity apply to it.
            PhotonTrackedTarget tag = singleTagUsed(estimate, used, layout, robotToCamera);
            if (tag == null) {
                return new PhotonVisionPoseEstimate(
                        pose, estimate.timestampSeconds, maxAmbiguity(used), used.isEmpty() ? 0 : 1);
            }
            return new PhotonVisionPoseEstimate(
                    pose,
                    estimate.timestampSeconds,
                    tag.getPoseAmbiguity(),
                    1,
                    tag.getBestCameraToTarget().getTranslation().getNorm());
        }

        private static boolean isMultiTag(PhotonPoseEstimator.PoseStrategy strategy) {
            return strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                    || strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_RIO
                    || strategy == PhotonPoseEstimator.PoseStrategy.CONSTRAINED_SOLVEPNP;
        }

        private static PhotonTrackedTarget singleTagUsed(
                org.photonvision.EstimatedRobotPose estimate,
                List<PhotonTrackedTarget> used,
                AprilTagFieldLayout layout,
                Transform3d robotToCamera) {
            if (used.isEmpty()) {
                return null;
            }
            if (estimate.strategy == PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY) {
                return lowestAmbiguityTarget(used);
            }
            if (estimate.strategy == PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE) {
                return used.get(0);
            }
            return targetProducing(estimate.estimatedPose, used, layout, robotToCamera);
        }

        /** Mirrors PhotonLib's lowest-ambiguity selection (first target with the smallest valid ambiguity). */
        static PhotonTrackedTarget lowestAmbiguityTarget(List<PhotonTrackedTarget> targets) {
            PhotonTrackedTarget lowest = null;
            double lowestScore = 10.0;
            for (PhotonTrackedTarget target : targets) {
                if (target == null) {
                    continue;
                }
                double score = target.getPoseAmbiguity();
                if (score != -1 && score < lowestScore) {
                    lowestScore = score;
                    lowest = target;
                }
            }
            return lowest;
        }

        /** Finds the tag whose best or alternate solve reproduces the estimated pose. */
        private static PhotonTrackedTarget targetProducing(
                Pose3d estimated,
                List<PhotonTrackedTarget> targets,
                AprilTagFieldLayout layout,
                Transform3d robotToCamera) {
            if (layout == null || robotToCamera == null || estimated == null) {
                return null;
            }
            for (PhotonTrackedTarget target : targets) {
                if (target == null || target.getFiducialId() == -1) {
                    continue;
                }
                Optional<Pose3d> tagPose = layout.getTagPose(target.getFiducialId());
                if (tagPose.isEmpty()) {
                    continue;
                }
                if (matches(estimated, tagPose.get(), target.getBestCameraToTarget(), robotToCamera)
                        || matches(estimated, tagPose.get(), target.getAlternateCameraToTarget(), robotToCamera)) {
                    return target;
                }
            }
            return null;
        }

        private static boolean matches(
                Pose3d estimated, Pose3d tagPose, Transform3d cameraToTarget, Transform3d robotToCamera) {
            if (cameraToTarget == null) {
                return false;
            }
            Pose3d candidate = tagPose.transformBy(cameraToTarget.inverse()).transformBy(robotToCamera.inverse());
            return candidate.getTranslation().getDistance(estimated.getTranslation()) <= SAME_POSE_TOLERANCE_METERS;
        }

        /**
         * Chooses the solve for one frame. A one-tag frame uses the heading-constrained trig solve when
         * enabled, and falls back to lowest ambiguity while no heading sample exists.
         */
        private Optional<org.photonvision.EstimatedRobotPose> estimateWithConfiguredStrategy(
                PhotonPipelineResult result) {
            int tagCount = result.getTargets() == null ? 0 : result.getTargets().size();
            PhotonPoseEstimator.PoseStrategy active =
                    strategyForFrame(strategy, singleTagUsesHeading.getAsBoolean(), tagCount);
            if (active == PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE) {
                return estimator.estimatePnpDistanceTrigSolvePose(result)
                        .or(() -> estimator.estimateLowestAmbiguityPose(result));
            }
            return switch (active) {
                case LOWEST_AMBIGUITY -> estimator.estimateLowestAmbiguityPose(result);
                case CLOSEST_TO_CAMERA_HEIGHT -> estimator.estimateClosestToCameraHeightPose(result);
                case CLOSEST_TO_REFERENCE_POSE -> closestToReference(result);
                case AVERAGE_BEST_TARGETS -> estimator.estimateAverageBestTargetsPose(result);
                case MULTI_TAG_PNP_ON_COPROCESSOR -> multiTagThenSingleTag(
                        estimator.estimateCoprocMultiTagPose(result),
                        () -> estimator.estimateLowestAmbiguityPose(result));
                case PNP_DISTANCE_TRIG_SOLVE -> estimator.estimatePnpDistanceTrigSolvePose(result);
                default -> estimator.estimateCoprocMultiTagPose(result)
                        .or(() -> estimator.estimateLowestAmbiguityPose(result));
            };
        }

        /**
         * Returns the solve to try first for a frame. Only the default coprocessor multi-tag strategy is
         * overridden, and only for frames with exactly one tag when single-tag heading is enabled.
         */
        static PhotonPoseEstimator.PoseStrategy strategyForFrame(
                PhotonPoseEstimator.PoseStrategy configured, boolean singleTagUsesHeading, int tagCount) {
            return singleTagUsesHeading
                    && configured == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR
                    && tagCount == 1
                    ? PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE
                    : configured;
        }

        private Optional<org.photonvision.EstimatedRobotPose> closestToReference(PhotonPipelineResult result) {
            Pose2d reference = referencePose == null ? null : referencePose.get();
            if (reference == null) {
                // Without a current estimate, "closest to the field origin" is meaningless.
                return estimator.estimateLowestAmbiguityPose(result);
            }
            return estimator.estimateClosestToReferencePose(result, new Pose3d(reference));
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

        private static double maxAmbiguity(List<PhotonTrackedTarget> targets) {
            if (targets == null || targets.isEmpty()) {
                return 0.0;
            }
            return targets.stream()
                    .filter(Objects::nonNull)
                    .mapToDouble(PhotonTrackedTarget::getPoseAmbiguity)
                    .filter(Double::isFinite)
                    .max()
                    .orElse(0.0);
        }
    }

    static <T> Optional<T> multiTagThenSingleTag(Optional<T> multiTag, Supplier<Optional<T>> singleTag) {
        Optional<T> primary = multiTag == null ? Optional.empty() : multiTag;
        return primary.isPresent() ? primary : Objects.requireNonNull(singleTag, "singleTag").get();
    }

    private static PhotonPoseClient defaultPoseClient(PhotonVisionDevice device, AprilTagFieldLayout fieldLayout) {
        PhotonPoseEstimator estimator = new PhotonPoseEstimator(
                Objects.requireNonNull(fieldLayout, "fieldLayout"),
                transform(CameraMountPose.identity()));
        return new PhotonPoseEstimatorClient(
                estimator,
                poseStrategyFor(device),
                device::mountPose,
                null,
                device::singleTagHeadingEnabled,
                () -> device.reportedRobotHeading().in(Radians));
    }

    /**
     * Maps the pose solve selected on a declaration to the PhotonLib strategy. Defaults to
     * coprocessor multi-tag when nothing was selected.
     *
     * @param device camera declaration
     * @return PhotonLib pose strategy
     */
    static PhotonPoseEstimator.PoseStrategy poseStrategyFor(PhotonVisionDevice device) {
        return device != null && device.poseStrategy() == PhotonVisionPoseStrategy.LOWEST_AMBIGUITY
                ? PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY
                : PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;
    }

    private static Transform3d transform(CameraMountPose mount) {
        CameraMountPose safe = mount == null || !mount.isFinite() ? CameraMountPose.identity() : mount;
        return new Transform3d(
                new Translation3d(safe.x().in(Meters), safe.y().in(Meters), safe.z().in(Meters)),
                new Rotation3d(
                        safe.roll().in(Radians),
                        safe.pitch().in(Radians),
                        safe.yaw().in(Radians)));
    }

    /**
     * One frame read from PhotonVision.
     *
     * @param secondsSinceLastPong age of the last time-sync confirmation when the frame was published; NaN if unknown
     */
    static record PhotonVisionResult(
            double timestampSeconds,
            double latencySeconds,
            List<PhotonVisionTarget> targets,
            Optional<PhotonVisionPoseEstimate> pose,
            double secondsSinceLastPong) {
        PhotonVisionResult {
            timestampSeconds = finiteOrZero(timestampSeconds);
            latencySeconds = Math.max(0.0, finiteOrZero(latencySeconds));
            targets = targets == null ? List.of() : List.copyOf(targets);
            pose = pose == null ? Optional.empty() : pose;
            secondsSinceLastPong = Double.isFinite(secondsSinceLastPong) && secondsSinceLastPong >= 0.0
                    ? secondsSinceLastPong
                    : Double.NaN;
        }

        PhotonVisionResult(
                double timestampSeconds,
                double latencySeconds,
                List<PhotonVisionTarget> targets,
                Optional<PhotonVisionPoseEstimate> pose) {
            this(timestampSeconds, latencySeconds, targets, pose, Double.NaN);
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
                    result.getTimestampSeconds(),
                    PhotonVisionCameraAdapter.latencySeconds(result),
                    targets,
                    pose,
                    PhotonVisionCameraAdapter.secondsSinceLastPong(result));
        }
    }

    /** PhotonLib reports metadata.timeSinceLastPong in microseconds. */
    private static double secondsSinceLastPong(PhotonPipelineResult result) {
        if (result == null || result.metadata == null) {
            return Double.NaN;
        }
        return result.metadata.timeSinceLastPong / 1.0e6;
    }

    /** Shared by every buffer of one adapter; holds the most recent frame and refresh times. */
    static final class ClockMonitor {
        private volatile PhotonVisionResult latest;
        private volatile double receivedAtSeconds = Double.NaN;
        private volatile double refreshedAtSeconds = Double.NaN;

        void refresh(PhotonVisionResult newFrame, double robotTimeSeconds) {
            if (newFrame != null) {
                latest = newFrame;
                receivedAtSeconds = robotTimeSeconds;
            }
            refreshedAtSeconds = robotTimeSeconds;
        }

        CameraClockStatus status() {
            return clockStatus(latest, receivedAtSeconds, refreshedAtSeconds);
        }
    }

    /**
     * Pose estimate seam.
     *
     * @param averageTargetDistanceMeters distance of the tags that produced the pose; NaN means
     *     "use the average of every visible target"
     */
    static record PhotonVisionPoseEstimate(
            PoseSnapshot pose,
            double timestampSeconds,
            double ambiguity,
            int targetCount,
            double averageTargetDistanceMeters) {
        PhotonVisionPoseEstimate {
            pose = Objects.requireNonNull(pose, "pose");
            timestampSeconds = finiteOrZero(timestampSeconds);
            ambiguity = Math.max(0.0, finiteOrZero(ambiguity));
            targetCount = Math.max(0, targetCount);
        }

        PhotonVisionPoseEstimate(PoseSnapshot pose, double timestampSeconds, double ambiguity, int targetCount) {
            this(pose, timestampSeconds, ambiguity, targetCount, Double.NaN);
        }
    }

    private static final class PhotonVisionMeasurementBuffer {
        private final Object source;
        private final PhotonClient client;
        private final PhotonPoseClient poseClient;
        private final ClockMonitor clockMonitor;
        private final DoubleSupplier robotClock;
        private PhotonVisionResult latest = new PhotonVisionResult(0.0, 0.0, List.of(), Optional.empty());

        private PhotonVisionMeasurementBuffer(
                Object source,
                PhotonClient client,
                PhotonPoseClient poseClient,
                ClockMonitor clockMonitor,
                DoubleSupplier robotClock) {
            this.source = source;
            this.client = Objects.requireNonNull(client, "client");
            this.poseClient = poseClient;
            this.clockMonitor = Objects.requireNonNull(clockMonitor, "clockMonitor");
            this.robotClock = Objects.requireNonNull(robotClock, "robotClock");
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
            if (!client.isConnected()) {
                throw new IllegalStateException("PhotonVision camera is disconnected: " + source);
            }
            double robotTimeSeconds = robotClock.getAsDouble();
            if (poseClient != null) {
                poseClient.beginCycle(robotTimeSeconds);
            }
            List<PhotonVisionResult> results = client.unreadResults(poseClient);
            PhotonVisionResult newFrame = null;
            if (!results.isEmpty()) {
                newFrame = results.get(results.size() - 1);
                latest = newFrame;
            }
            clockMonitor.refresh(newFrame, robotTimeSeconds);
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
