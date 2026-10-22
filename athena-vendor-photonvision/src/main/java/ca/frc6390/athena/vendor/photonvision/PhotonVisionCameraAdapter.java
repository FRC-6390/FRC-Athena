package ca.frc6390.athena.vendor.photonvision;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import java.util.ArrayDeque;
import java.util.Deque;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.concurrent.atomic.AtomicLong;
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
import edu.wpi.first.wpilibj.RobotBase;
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
                realCameraClient(device),
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
                realCameraClient(device),
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
                realCameraClient(device),
                new PhotonPoseEstimatorClient(
                        poseEstimator, poseStrategy, device::mountPose, referencePose, device::singleTagHeadingEnabled,
                        () -> device.reportedRobotHeading().in(Radians)),
                Timer::getFPGATimestamp);
    }

    /**
     * Opens the PhotonLib camera for a device. On the real robot the camera is read by a background
     * thread; in simulation every read happens on the caller's thread, as before.
     */
    private static PhotonCameraClient realCameraClient(PhotonVisionDevice device) {
        PhotonCamera camera = new PhotonCamera(Objects.requireNonNull(device, "device").name());
        return new PhotonCameraClient(camera, !RobotBase.isSimulation());
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
        poseClient.beginCycle(robotClock.getAsDouble());
        List<PhotonVisionResult> results = client.unreadResults(poseClient);
        if (client.separatePoseResults()) results = client.completedPoseResults();
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

    /**
     * Returns how many decoded frames were dropped by the background reader or replaced in the latest-only
     * pending pose slot. Always 0 when frames are read synchronously (simulation).
     *
     * @return dropped frame count
     */
    public long droppedFrames() {
        return client instanceof PhotonCameraClient camera ? camera.droppedFrames() : 0L;
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

        default boolean separatePoseResults() { return false; }

        default List<PhotonVisionResult> completedPoseResults() { return List.of(); }

        @Override
        void close();
    }

    interface PhotonPoseClient {
        Optional<PhotonVisionPoseEstimate> estimate(PhotonPipelineResult result);

        /** Captures an immutable solve context on the robot thread; the returned solver runs on the worker. */
        default PhotonPoseClient snapshot() { return this; }

        /**
         * Called once per robot cycle before frames are read, with the robot time of that cycle.
         *
         * @param robotTimeSeconds robot time of the cycle; NaN when no robot clock is available
         */
        default void beginCycle(double robotTimeSeconds) {
        }
    }

    /**
     * Reads one PhotonLib camera. Synchronous mode reads on the caller's thread. Background mode runs a daemon
     * reader that decodes the newest frame's timing and targets into a bounded queue. Pose work runs on a separate
     * worker using robot-thread snapshots, with one active and one latest pending frame. Target refresh never waits
     * for a solve, and completed poses retain their own frame's timing and target metadata.
     */
    static final class PhotonCameraClient implements PhotonClient {
        /** Decoded frames held for the robot thread; the oldest is dropped when a new one would exceed this. */
        static final int QUEUE_CAPACITY = 8;
        // Polls stay well under the 20 ms robot cycle without spinning the second core on empty reads.
        private static final long POLL_MILLIS = 10L;

        /** The PhotonLib calls this client makes. A seam so tests can feed frames without NetworkTables. */
        interface FrameSource {
            List<PhotonPipelineResult> unreadResults();

            boolean isConnected();

            void close();
        }

        private final FrameSource source;
        private final Deque<QueuedFrame> queue = new ArrayDeque<>();
        private final AtomicLong droppedFrames = new AtomicLong();
        private final Thread reader;
        private final Thread solver;
        private final Object poseLock = new Object();
        private PoseJob pendingPose;
        private final java.util.concurrent.atomic.AtomicReference<QueuedFrame> completedPose =
                new java.util.concurrent.atomic.AtomicReference<>();
        private volatile boolean closed;

        PhotonCameraClient(PhotonCamera camera, boolean background) {
            this(fromCamera(Objects.requireNonNull(camera, "camera")), background);
        }

        PhotonCameraClient(FrameSource source, boolean background) {
            this.source = Objects.requireNonNull(source, "source");
            if (background) {
                reader = new Thread(this::readLoop, "athena-photonvision-reader");
                reader.setDaemon(true);
                solver = new Thread(this::solveLoop, "athena-photonvision-solver");
                solver.setDaemon(true);
                solver.setPriority(Thread.MIN_PRIORITY);
                solver.start();
                reader.start();
            } else {
                reader = null;
                solver = null;
            }
        }

        private static FrameSource fromCamera(PhotonCamera camera) {
            return new FrameSource() {
                @Override
                public List<PhotonPipelineResult> unreadResults() {
                    return camera.getAllUnreadResults();
                }

                @Override
                public boolean isConnected() {
                    return camera.isConnected();
                }

                @Override
                public void close() {
                    camera.close();
                }
            };
        }

        @Override
        public List<PhotonVisionResult> unreadResults(PhotonPoseClient poseClient) {
            if (closed) return List.of();
            if (reader == null) {
                List<PhotonPipelineResult> results = source.unreadResults();
                if (results.isEmpty()) return List.of();
                return List.of(PhotonVisionResult.fromPhoton(results.get(results.size() - 1), poseClient));
            }
            QueuedFrame frame = drain();
            if (frame == null) return List.of();
            if (poseClient != null) {
                PhotonPoseClient captured = poseClient.snapshot();
                synchronized (poseLock) {
                    if (!closed) {
                        if (pendingPose != null) droppedFrames.incrementAndGet();
                        pendingPose = new PoseJob(frame, captured);
                        poseLock.notifyAll();
                    }
                }
            }
            return List.of(frame.converted());
        }

        @Override public boolean separatePoseResults() { return reader != null; }

        @Override
        public List<PhotonVisionResult> completedPoseResults() {
            if (closed) return List.of();
            QueuedFrame completed = completedPose.getAndSet(null);
            if (completed == null) return List.of();
            if (completed.failure() != null) throw completed.failure();
            return List.of(completed.converted());
        }

        private void solveLoop() {
            while (!closed) {
                PoseJob job;
                synchronized (poseLock) {
                    while (pendingPose == null && !closed) {
                        try {
                            poseLock.wait();
                        } catch (InterruptedException exception) {
                            return;
                        }
                    }
                    if (closed) return;
                    job = pendingPose;
                    pendingPose = null;
                }
                QueuedFrame result;
                try {
                    result = new QueuedFrame(job.frame().raw(), job.frame().converted()
                            .withPose(job.client().estimate(job.frame().raw())), null);
                } catch (RuntimeException failure) {
                    result = QueuedFrame.failed(failure);
                }
                if (!closed) completedPose.set(result);
            }
        }

        private record PoseJob(QueuedFrame frame, PhotonPoseClient client) { }

        /** Takes every queued frame in arrival order and returns the newest; rethrows a reader failure first. */
        private QueuedFrame drain() {
            QueuedFrame newest = null;
            synchronized (queue) {
                while (!queue.isEmpty()) {
                    QueuedFrame next = queue.poll();
                    if (next.failure() != null) {
                        throw next.failure();
                    }
                    newest = next;
                }
            }
            return newest;
        }

        private void readLoop() {
            while (!closed) {
                try {
                    List<PhotonPipelineResult> results = source.unreadResults();
                    if (results.isEmpty()) {
                        Thread.sleep(POLL_MILLIS);
                        continue;
                    }
                    // Like the synchronous path, only the newest frame of a batch is used.
                    offer(QueuedFrame.of(results.get(results.size() - 1)));
                } catch (InterruptedException exception) {
                    return;
                } catch (RuntimeException exception) {
                    if (closed) {
                        return;
                    }
                    offer(QueuedFrame.failed(exception));
                    try {
                        Thread.sleep(POLL_MILLIS);
                    } catch (InterruptedException interrupted) {
                        return;
                    }
                }
            }
        }

        private void offer(QueuedFrame frame) {
            synchronized (queue) {
                if (queue.size() >= QUEUE_CAPACITY) {
                    queue.poll();
                    droppedFrames.incrementAndGet();
                }
                queue.offer(frame);
            }
        }

        long droppedFrames() {
            return droppedFrames.get();
        }

        @Override
        public boolean isConnected() {
            return source.isConnected();
        }

        @Override
        public void close() {
            if (closed) return;
            closed = true;
            if (reader != null) {
                reader.interrupt();
            }
            if (solver != null) solver.interrupt();
            synchronized (poseLock) { pendingPose = null; poseLock.notifyAll(); }
            completedPose.set(null);
            source.close();
        }

        /**
         * One frame from the reader thread: the PhotonLib result and its robot-independent conversion, or the
         * failure that the robot thread must rethrow in its own refresh.
         */
        private record QueuedFrame(PhotonPipelineResult raw, PhotonVisionResult converted, RuntimeException failure) {
            static QueuedFrame of(PhotonPipelineResult raw) {
                try {
                    return new QueuedFrame(raw, PhotonVisionResult.fromTargets(raw), null);
                } catch (RuntimeException exception) {
                    return failed(exception);
                }
            }

            static QueuedFrame failed(RuntimeException exception) {
                return new QueuedFrame(null, null, exception);
            }
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
        private static final int MAX_HEADING_SAMPLES = 256;
        private final Deque<HeadingSample> headingHistory = new ArrayDeque<>();
        private SolveContext cycleContext;

        private record HeadingSample(double timestampSeconds, Rotation2d heading) { }
        private record SolveContext(Transform3d mount, Pose2d reference, boolean headingEnabled,
                List<HeadingSample> headings) { }

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
            cycleContext = captureContext(robotTimeSeconds);
        }

        private SolveContext captureContext(double robotTimeSeconds) {
            Pose2d reference = referencePose == null ? null : referencePose.get();
            boolean headingEnabled = singleTagUsesHeading.getAsBoolean();
            if (Double.isFinite(robotTimeSeconds) && headingEnabled) {
                Rotation2d heading = robotHeading(reference);
                if (heading != null) {
                    if (!headingHistory.isEmpty() && headingHistory.peekLast().timestampSeconds() > robotTimeSeconds) {
                        headingHistory.clear();
                    } else if (!headingHistory.isEmpty()
                            && headingHistory.peekLast().timestampSeconds() == robotTimeSeconds) {
                        headingHistory.removeLast();
                    }
                    headingHistory.addLast(new HeadingSample(robotTimeSeconds, heading));
                    // PhotonLib keeps one second of heading samples. Preserve that interval while bounding
                    // pathological caller rates as well as the pending frame queue.
                    while (headingHistory.size() > MAX_HEADING_SAMPLES
                            || headingHistory.peekFirst().timestampSeconds() < robotTimeSeconds - 1.0) {
                        headingHistory.removeFirst();
                    }
                }
            }
            return new SolveContext(mountPose == null ? null : transform(mountPose.get()), reference,
                    headingEnabled, List.copyOf(headingHistory));
        }

        private Rotation2d robotHeading(Pose2d reference) {
            if (reference != null) return reference.getRotation();
            double reported = reportedHeadingRadians.getAsDouble();
            if (Double.isFinite(reported)) return new Rotation2d(reported);
            Supplier<Rotation2d> source = robotHeadingSource;
            return source == null ? null : source.get();
        }

        @Override
        public PhotonPoseClient snapshot() {
            SolveContext captured = cycleContext == null ? captureContext(Double.NaN) : cycleContext;
            return result -> estimate(result, captured);
        }

        @Override
        public Optional<PhotonVisionPoseEstimate> estimate(PhotonPipelineResult result) {
            // Deterministic synchronous adapters (simulation) still capture and solve on their caller.
            return estimate(result, captureContext(Double.NaN));
        }

        private Optional<PhotonVisionPoseEstimate> estimate(PhotonPipelineResult result, SolveContext captured) {
            if (captured.mount() != null) estimator.setRobotToCameraTransform(captured.mount());
            List<HeadingSample> headings = captured.headings();
            if (!headings.isEmpty()) {
                HeadingSample first = headings.get(0);
                estimator.resetHeadingData(first.timestampSeconds(), first.heading());
                for (int index = 1; index < headings.size(); index++) {
                    HeadingSample sample = headings.get(index);
                    estimator.addHeadingData(sample.timestampSeconds(), sample.heading());
                }
            }
            return estimateWithConfiguredStrategy(result, captured).map(estimate -> toEstimate(
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
                PhotonPipelineResult result, SolveContext captured) {
            int tagCount = result.getTargets() == null ? 0 : result.getTargets().size();
            PhotonPoseEstimator.PoseStrategy active =
                    strategyForFrame(strategy, captured.headingEnabled(), tagCount);
            if (active == PhotonPoseEstimator.PoseStrategy.PNP_DISTANCE_TRIG_SOLVE) {
                return estimator.estimatePnpDistanceTrigSolvePose(result)
                        .or(() -> estimator.estimateLowestAmbiguityPose(result));
            }
            return switch (active) {
                case LOWEST_AMBIGUITY -> estimator.estimateLowestAmbiguityPose(result);
                case CLOSEST_TO_CAMERA_HEIGHT -> estimator.estimateClosestToCameraHeightPose(result);
                case CLOSEST_TO_REFERENCE_POSE -> closestToReference(result, captured.reference());
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

        private Optional<org.photonvision.EstimatedRobotPose> closestToReference(PhotonPipelineResult result, Pose2d reference) {
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
            return fromTargets(result).withPose(pose);
        }

        /**
         * The part of a frame that needs no robot state: timing and targets, with no pose. Safe to compute on
         * the reader thread.
         */
        static PhotonVisionResult fromTargets(PhotonPipelineResult result) {
            if (result == null) {
                return new PhotonVisionResult(0.0, 0.0, List.of(), Optional.empty());
            }
            List<PhotonVisionTarget> targets = result.hasTargets()
                    ? result.getTargets().stream().map(PhotonVisionCameraAdapter::targetFromPhoton).toList()
                    : List.of();
            return new PhotonVisionResult(
                    result.getTimestampSeconds(),
                    PhotonVisionCameraAdapter.latencySeconds(result),
                    targets,
                    Optional.empty(),
                    PhotonVisionCameraAdapter.secondsSinceLastPong(result));
        }

        PhotonVisionResult withPose(Optional<PhotonVisionPoseEstimate> pose) {
            return new PhotonVisionResult(timestampSeconds, latencySeconds, targets, pose, secondsSinceLastPong);
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
        private PhotonVisionResult latestPose = latest;

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
            return poseMeasurementsFromResult(latestPose, source);
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
                if (!client.separatePoseResults()) latestPose = newFrame;
            }
            if (client.separatePoseResults()) {
                List<PhotonVisionResult> poses = client.completedPoseResults();
                if (!poses.isEmpty()) latestPose = poses.get(poses.size() - 1);
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
