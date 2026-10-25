package ca.frc6390.athena.localization.pipeline;

import static edu.wpi.first.units.Units.MetersPerSecond;

import ca.frc6390.athena.drivetrain.swerve.SwerveOdometry;
import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.HardwareMeasurementSignal;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.control.VelocityFrame;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import ca.frc6390.athena.runtime.measurement.MeasurementSignal;
import ca.frc6390.athena.runtime.measurement.MeasurementSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import ca.frc6390.athena.runtime.measurement.ResettablePoseSignal;
import ca.frc6390.athena.localization.pipeline.VisionDiagnostics.Verdict;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.IntegerPublisher;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Optional;
import java.util.Set;
import ca.frc6390.athena.vision.device.RobotHeadingAware;
import java.util.function.BooleanSupplier;

import static edu.wpi.first.units.Units.Hertz;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;

/** One inspectable pose-processing node in a localization graph. */
public final class Localization implements PoseSignal {
    enum Strategy {
        FILTER,
        LATEST_VALID,
        WEIGHTED_AVERAGE,
        COVARIANCE_INTERSECTION,
        KALMAN
    }

    private static final MeasurementStdDevs DEFAULT_STATE_STD_DEVS = MeasurementStdDevs.of(Meters.of(0.1), Meters.of(0.1), Radians.of(0.1));
    private static final MeasurementStdDevs DEFAULT_VISION_STD_DEVS = MeasurementStdDevs.of(Meters.of(0.9), Meters.of(0.9), Radians.of(0.9));
    private static final double TWO_PI = Math.PI * 2.0;

    private final Strategy strategy;
    private final List<PoseSignal> inputs;
    private final List<LocalizationFilter> filters;
    private final String debugName;
    private final boolean publishNetworkTables;
    private final double groupWindowSeconds;
    private final double maxTranslationDisagreementMeters;
    private final double maxHeadingDisagreementRadians;
    private final int minimumSourceCount;
    private final double innovationGate;
    private final MeasurementStdDevs stateStdDevs;
    private final MeasurementStdDevs defaultVisionStdDevs;
    private final KalmanTuning tuning;
    private final State state;

    Localization(Strategy strategy) {
        this(
                strategy,
                List.of(),
                List.of(),
                "",
                false,
                0.03,
                Double.POSITIVE_INFINITY,
                Double.POSITIVE_INFINITY,
                1,
                Double.POSITIVE_INFINITY,
                DEFAULT_STATE_STD_DEVS,
                DEFAULT_VISION_STD_DEVS,
                KalmanTuning.NONE,
                new State());
    }

    private Localization(
            Strategy strategy,
            List<PoseSignal> inputs,
            List<LocalizationFilter> filters,
            String debugName,
            boolean publishNetworkTables,
            double groupWindowSeconds,
            double maxTranslationDisagreementMeters,
            double maxHeadingDisagreementRadians,
            int minimumSourceCount,
            double innovationGate,
            MeasurementStdDevs stateStdDevs,
            MeasurementStdDevs defaultVisionStdDevs,
            KalmanTuning tuning,
            State state) {
        this.strategy = Objects.requireNonNull(strategy, "strategy");
        this.inputs = List.copyOf(inputs);
        this.filters = List.copyOf(filters);
        this.debugName = debugName == null ? "" : debugName;
        this.publishNetworkTables = publishNetworkTables;
        this.groupWindowSeconds = groupWindowSeconds;
        this.maxTranslationDisagreementMeters = maxTranslationDisagreementMeters;
        this.maxHeadingDisagreementRadians = maxHeadingDisagreementRadians;
        this.minimumSourceCount = minimumSourceCount;
        this.innovationGate = innovationGate;
        this.stateStdDevs = Objects.requireNonNull(stateStdDevs, "stateStdDevs");
        this.defaultVisionStdDevs = Objects.requireNonNull(defaultVisionStdDevs, "defaultVisionStdDevs");
        this.tuning = Objects.requireNonNull(tuning, "tuning");
        this.state = Objects.requireNonNull(state, "state");
    }

    /** Adds pose-producing inputs to this node. */
    public Localization input(PoseSignal... addedInputs) {
        List<PoseSignal> next = new ArrayList<>(inputs);
        if (addedInputs != null) {
            for (PoseSignal input : addedInputs) {
                next.add(Objects.requireNonNull(input, "input"));
            }
        }
        return copy(next, filters);
    }

    /** Adds measurement filters to this node. */
    public Localization filter(LocalizationFilter... addedFilters) {
        List<LocalizationFilter> next = new ArrayList<>(filters);
        if (addedFilters != null) {
            for (LocalizationFilter filter : addedFilters) {
                next.add(Objects.requireNonNull(filter, "filter"));
            }
        }
        return copy(inputs, next);
    }

    public Localization name(String name) {
        return configured(name, publishNetworkTables, groupWindowSeconds, maxTranslationDisagreementMeters,
                maxHeadingDisagreementRadians, innovationGate, stateStdDevs, defaultVisionStdDevs);
    }

    public Localization publishNetworkTables() {
        return publishNetworkTables(true);
    }

    public Localization publishNetworkTables(boolean enabled) {
        return configured(debugName, enabled, groupWindowSeconds, maxTranslationDisagreementMeters,
                maxHeadingDisagreementRadians, innovationGate, stateStdDevs, defaultVisionStdDevs);
    }

    public Localization groupWithin(Time window) {
        return configured(debugName, publishNetworkTables, nonNegative(window.in(Seconds), "Group window"),
                maxTranslationDisagreementMeters, maxHeadingDisagreementRadians, innovationGate,
                stateStdDevs, defaultVisionStdDevs);
    }

    public Localization maxTranslationDisagreement(Distance distance) {
        return configured(debugName, publishNetworkTables, groupWindowSeconds,
                nonNegative(distance.in(Meters), "Translation disagreement"), maxHeadingDisagreementRadians,
                innovationGate, stateStdDevs, defaultVisionStdDevs);
    }

    public Localization maxHeadingDisagreement(Angle heading) {
        return configured(debugName, publishNetworkTables, groupWindowSeconds,
                maxTranslationDisagreementMeters, nonNegative(heading.in(Radians), "Heading disagreement"),
                innovationGate, stateStdDevs, defaultVisionStdDevs);
    }

    /** Requires a fused camera group to contain at least this many distinct sources. */
    public Localization minimumSourceCount(int count) {
        if (count < 1) {
            throw new IllegalArgumentException("Minimum source count must be at least one.");
        }
        return new Localization(strategy, inputs, filters, debugName, publishNetworkTables,
                groupWindowSeconds, maxTranslationDisagreementMeters, maxHeadingDisagreementRadians,
                count, innovationGate, stateStdDevs, defaultVisionStdDevs, tuning, state);
    }

    /** Rejects vision whose squared residual divided by measurement variance exceeds this value. */
    public Localization maxNormalizedVisionResidual(double maximum) {
        return configured(debugName, publishNetworkTables, groupWindowSeconds,
                maxTranslationDisagreementMeters, maxHeadingDisagreementRadians,
                nonNegative(maximum, "Normalized vision residual"), stateStdDevs, defaultVisionStdDevs);
    }

    public Localization stateStdDevs(Distance translation, Angle heading) {
        return configured(debugName, publishNetworkTables, groupWindowSeconds,
                maxTranslationDisagreementMeters, maxHeadingDisagreementRadians, innovationGate,
                stdDevsOf(translation, heading), defaultVisionStdDevs);
    }

    public Localization defaultVisionStdDevs(Distance translation, Angle heading) {
        return configured(debugName, publishNetworkTables, groupWindowSeconds,
                maxTranslationDisagreementMeters, maxHeadingDisagreementRadians, innovationGate,
                stateStdDevs, stdDevsOf(translation, heading));
    }

    private static MeasurementStdDevs stdDevsOf(Distance translation, Angle heading) {
        double translationMeters = translation.in(Meters);
        return MeasurementStdDevs.of(Meters.of(translationMeters), Meters.of(translationMeters), Radians.of(heading.in(Radians)));
    }

    /**
     * Scales vision standard deviations with robot motion (Kalman nodes only):
     * {@code multiplier = 1 + perMetersPerSecond * |v| + perRadiansPerSecond * |omega|}. The speeds are the frame's
     * own speeds when it reports them, otherwise the odometry speeds at the refresh. A frame taken while driving
     * is trusted less, so a motion-blurred or late frame cannot pull the estimate as hard.
     *
     * @param perMetersPerSecond std dev growth per m/s of translation speed (0 disables)
     * @param perRadiansPerSecond std dev growth per rad/s of rotation speed (0 disables)
     * @return updated node
     */
    public Localization visionStdDevSpeedScaling(double perMetersPerSecond, double perRadiansPerSecond) {
        return withTuning(new KalmanTuning(
                nonNegative(perMetersPerSecond, "Speed scaling"),
                nonNegative(perRadiansPerSecond, "Angular speed scaling"),
                tuning.maxAngularVelocity(),
                tuning.seedWhileDisabled(),
                tuning.visionHeadingWhile()));
    }

    /**
     * Rejects vision frames taken while the robot rotates faster than this (Kalman nodes only). The reason reads
     * {@code spinning 4.1 rad/s > 3.0}. Uses the frame's own speeds when reported, otherwise odometry.
     *
     * @param radiansPerSecond maximum rotation speed in rad/s; infinity disables the gate
     * @return updated node
     */
    public Localization maxVisionAngularVelocity(AngularVelocity limit) {
        return withTuning(new KalmanTuning(
                tuning.speedScaleLinear(),
                tuning.speedScaleAngular(),
                nonNegative(limit.in(RadiansPerSecond), "Maximum angular velocity"),
                tuning.seedWhileDisabled(),
                tuning.visionHeadingWhile()));
    }

    /**
     * While {@code disabled} reports true, a multi-tag frame (two or more tags) that passes the filters resets the
     * estimate to the vision pose, instead of a Kalman update, at most once per 0.25 s. Only the parts the frame
     * trusts are taken: a translation-only frame keeps the current heading, a heading-only frame the current
     * translation. Single-tag frames still take the normal update. This puts the pose right before autonomous
     * starts.
     *
     * <p>Independently of this option, a Kalman node that has not been reset seeds itself once from the first
     * self-consistent vision frames, single-tag included.</p>
     *
     * <p>The supplier is the caller's, because the localization runtime has no mode of its own. A robot passes
     * {@code DriverStation::isDisabled}. Kalman nodes only.</p>
     *
     * @param disabled true while the robot is disabled
     * @return updated node
     */
    public Localization seedFromVisionWhileDisabled(BooleanSupplier disabled) {
        return withTuning(new KalmanTuning(
                tuning.speedScaleLinear(),
                tuning.speedScaleAngular(),
                tuning.maxAngularVelocity(),
                Objects.requireNonNull(disabled, "disabled"),
                tuning.visionHeadingWhile()));
    }

    /**
     * Uses camera heading only while {@code condition} reads true, for example {@code DriverStation::isDisabled};
     * the gyro holds heading the rest of the time. Kalman nodes only.
     *
     * <p>Camera heading is usually a few degrees off (mount yaw, single-tag solves), and the estimator follows
     * it within a fraction of a second whatever the std devs, so paths and aiming turn the robot to match a
     * wrong heading. With this option frames keep correcting translation while the condition is false; their
     * heading is ignored. Heading is still taken from vision until the estimate has one, from a reset or a
     * seed, so a robot that is enabled before it ever saw a tag can still find its heading.</p>
     *
     * @param condition true while camera heading may correct the estimate
     * @return updated node
     */
    public Localization visionHeadingWhile(BooleanSupplier condition) {
        return withTuning(new KalmanTuning(
                tuning.speedScaleLinear(),
                tuning.speedScaleAngular(),
                tuning.maxAngularVelocity(),
                tuning.seedWhileDisabled(),
                Objects.requireNonNull(condition, "condition")));
    }

    private Localization withTuning(KalmanTuning next) {
        return new Localization(strategy, inputs, filters, debugName, publishNetworkTables,
                groupWindowSeconds, maxTranslationDisagreementMeters, maxHeadingDisagreementRadians,
                minimumSourceCount, innovationGate, stateStdDevs, defaultVisionStdDevs, next, new State());
    }

    /**
     * Returns the status of every camera this Kalman node has received frames from, as of its last refresh.
     * Covers the filter stages upstream of this node as well as its own filters and Kalman gates. Empty for other
     * strategies and before the first refresh.
     *
     * @return one status per camera name, in first-seen order
     */
    public List<VisionSourceStatus> visionStatus() {
        synchronized (state) {
            if (strategy != Strategy.KALMAN) {
                return List.of();
            }
            return state.vision.snapshot(state.visionNow);
        }
    }

    /** Refreshes this node and all of its inputs exactly once for the runtime timestamp. */
    public PoseSignal refresh(ActionContext context, double timestampSeconds, double dtSeconds) {
        synchronized (state) {
            if (Double.compare(state.lastRefreshTimestamp, timestampSeconds) == 0) {
                return state.output;
            }
            refreshInputs(context, timestampSeconds, dtSeconds);
            evaluate(timestampSeconds);
            reportHeadingToCameras(timestampSeconds);
            publishPose();
            state.lastRefreshTimestamp = timestampSeconds;
            state.runtimeOwned = true;
            return state.output;
        }
    }

    @Override
    public List<Measurement> measurements() {
        synchronized (state) {
            if (!state.runtimeOwned && strategy != Strategy.KALMAN) {
                evaluate(Double.NaN);
            }
            return state.output.measurements();
        }
    }

    /** Returns the newest pose sample of this node's output, from the cycle result instead of a rescan. */
    @Override
    public Optional<PoseMeasurementSample> measurement() {
        synchronized (state) {
            if (!state.runtimeOwned && strategy != Strategy.KALMAN) {
                evaluate(Double.NaN);
            }
            return Optional.ofNullable(state.output.latestPose());
        }
    }

    public List<Measurement> acceptedMeasurements() {
        synchronized (state) {
            return state.accepted;
        }
    }

    public List<Measurement> rejectedMeasurements() {
        synchronized (state) {
            return state.rejected;
        }
    }

    /** Returns the current estimate using WPILib's standard pose type. */
    public Pose2d pose2d() {
        PoseSnapshot pose = poseSnapshot();
        return new Pose2d(pose.xMeters(), pose.yMeters(), new Rotation2d(pose.headingRadians()));
    }

    /**
     * Returns whether this estimate has been seeded: a {@link #reset(PoseSnapshot)} has been applied, or, for
     * Kalman nodes, a vision frame has passed this node's filters. False at boot. Once true it stays true.
     *
     * <p>Robot code uses this to decide whether the field pose can be trusted yet.</p>
     *
     * @return true once the pose has been seeded
     */
    public boolean seeded() {
        synchronized (state) {
            return state.seeded;
        }
    }

    public ActionBinding reset(PoseSnapshot pose) {
        Objects.requireNonNull(pose, "pose");
        return context -> {
            synchronized (state) {
                resetInputs(pose);
                state.seeded = true;
                state.pendingReset = pose;
                state.output = CycleResult.of(List.of(outputMeasurement(
                        pose, RobotVelocity.zero(), 0.0, stateStdDevs, this)));
                state.accepted = List.of();
                state.rejected = List.of();
                state.lastRefreshTimestamp = Double.NaN;
                state.kalman = null;
            }
        };
    }

    /** Resets this localization chain from a WPILib pose. */
    public ActionBinding reset(Pose2d pose) {
        Objects.requireNonNull(pose, "pose");
        return reset(new PoseSnapshot(pose.getX(), pose.getY(), pose.getRotation().getRadians()));
    }

    public ActionBinding resetTo(PoseSignal other) {
        Objects.requireNonNull(other, "other");
        return reset(other.pose());
    }

    public String strategyName() {
        return strategy.name().toLowerCase(java.util.Locale.ROOT);
    }

    public List<PoseSignal> inputs() {
        return inputs;
    }

    public List<LocalizationFilter> filters() {
        return filters;
    }

    public String debugName() {
        return debugName;
    }

    public boolean publishNetworkTablesEnabled() {
        return publishNetworkTables;
    }

    /**
     * Returns the runtime timestamp of the cycle currently being evaluated by this node.
     *
     * <p>Filters can use this as "now" (for example to reject stale camera frames). It is
     * {@code NaN} seconds when the node is evaluated outside a runtime refresh.</p>
     *
     * @return cycle timestamp, or NaN seconds
     */
    public Time cycleTimestamp() {
        return Seconds.of(evaluatingSeconds());
    }

    /** Primitive form of {@link #cycleTimestamp()} for per-frame filters in this package. */
    double evaluatingSeconds() {
        synchronized (state) {
            return state.evaluatingTimestamp;
        }
    }

    private void refreshInputs(ActionContext context, double timestampSeconds, double dtSeconds) {
        for (MeasurementSignal upstream : refreshableUpstreams()) {
            if (upstream instanceof Localization localization) {
                localization.refresh(context, timestampSeconds, dtSeconds);
            } else if (upstream instanceof HardwareMeasurementSignal hardware) {
                hardware.refresh(context, timestampSeconds, dtSeconds);
            }
        }
    }

    /**
     * Returns the upstream nodes this node must refresh before evaluating, in input order.
     *
     * <p>Derived signals such as {@code translationOnly()}, {@code headingOnly()} or tag covariance
     * wrappers are not refreshable themselves; they are walked through {@link MeasurementSignal#sources()}
     * so the {@link Localization} or hardware signal they wrap is refreshed first in the same cycle.
     * Without this, a node fed by {@code other.translationOnly()} would read {@code other}'s
     * previous-cycle output whenever the runtime refreshed it after this node.</p>
     */
    private List<MeasurementSignal> refreshableUpstreams() {
        List<MeasurementSignal> cached = state.refreshables;
        if (cached != null) {
            return cached;
        }
        List<MeasurementSignal> found = new ArrayList<>();
        Set<MeasurementSignal> visited = java.util.Collections.newSetFromMap(new IdentityHashMap<>());
        for (PoseSignal input : inputs) {
            collectRefreshable(input, found, visited);
        }
        state.refreshables = List.copyOf(found);
        return state.refreshables;
    }

    /**
     * A Kalman node gives its heading to every upstream camera that solves with the robot heading
     * (PhotonVision's heading-constrained single-tag solve), so robot code does not wire a heading source.
     */
    private void reportHeadingToCameras(double timestampSeconds) {
        if (strategy != Strategy.KALMAN) {
            return;
        }
        List<RobotHeadingAware> cameras = state.headingCameras;
        if (cameras == null) {
            List<RobotHeadingAware> found = new ArrayList<>();
            collectHeadingCameras(this, found, java.util.Collections.newSetFromMap(new IdentityHashMap<>()));
            cameras = state.headingCameras = List.copyOf(found);
        }
        if (cameras.isEmpty()) {
            return;
        }
        // poseSnapshot() of this Kalman node, read from the cycle result: the newest pose, or the origin.
        PoseMeasurementSample latest = state.output.latestPose();
        PoseSnapshot pose = latest == null ? null : latest.pose();
        double heading = pose == null ? 0.0 : pose.headingRadians();
        if (!Double.isFinite(heading)) {
            return;
        }
        Angle reported = Radians.of(heading);
        for (RobotHeadingAware camera : cameras) {
            camera.reportRobotHeading(timestampSeconds, reported);
        }
    }

    private static void collectHeadingCameras(
            MeasurementSignal signal, List<RobotHeadingAware> found, Set<Object> visited) {
        if (signal == null || !visited.add(signal)) {
            return;
        }
        if (signal instanceof ca.frc6390.athena.vision.signal.PoseSignal vision
                && vision.camera() instanceof RobotHeadingAware camera
                && visited.add(camera)) {
            found.add(camera);
        }
        if (signal instanceof Localization localization) {
            for (PoseSignal input : localization.inputs) {
                collectHeadingCameras(input, found, visited);
            }
        }
        for (MeasurementSignal source : signal.sources()) {
            collectHeadingCameras(source, found, visited);
        }
    }

    private static void collectRefreshable(
            MeasurementSignal signal,
            List<MeasurementSignal> found,
            Set<MeasurementSignal> visited) {
        if (signal == null || !visited.add(signal)) {
            return;
        }
        if (signal instanceof Localization || signal instanceof HardwareMeasurementSignal) {
            found.add(signal);
            return;
        }
        for (MeasurementSignal source : signal.sources()) {
            collectRefreshable(source, found, visited);
        }
    }

    private void evaluate(double timestampSeconds) {
        state.evaluatingTimestamp = timestampSeconds;
        // Allocated on first use: a cycle without measurements keeps the shared empty lists.
        List<Measurement> accepted = null;
        List<Rejection> rejections = null;
        SwerveOdometry odometry = null;
        for (int inputIndex = 0; inputIndex < inputs.size(); inputIndex++) {
            PoseSignal input = inputs.get(inputIndex);
            if (strategy == Strategy.KALMAN && input instanceof SwerveOdometry swerveOdometry) {
                if (odometry != null && odometry != swerveOdometry) {
                    throw new IllegalStateException("Kalman localization accepts exactly one odometry input.");
                }
                odometry = swerveOdometry;
                continue;
            }
            // Indexed loops: measurement lists are small and random access, and this runs every cycle.
            List<Measurement> measurements = input.measurements();
            for (int index = 0; index < measurements.size(); index++) {
                Measurement measurement = measurements.get(index);
                PoseSnapshot pose = pose(measurement);
                if (pose == null) {
                    rejections = added(rejections, new Rejection(measurement, "not a pose measurement"));
                    continue;
                }
                String reason = rejection(measurement, pose);
                if (reason == null) {
                    accepted = added(accepted, measurement);
                } else {
                    rejections = added(rejections, new Rejection(measurement, reason));
                }
            }
        }
        state.accepted = accepted == null ? List.of() : List.copyOf(accepted);
        if (strategy == Strategy.KALMAN && accepted != null) {
            // Kalman inputs are vision only (odometry is skipped above), so an accepted frame seeds the pose.
            state.seeded = true;
        }
        state.rejections = rejections == null ? List.of() : List.copyOf(rejections);
        state.rejected = rejections == null ? List.of() : rejectedMeasurements(rejections);
        List<Measurement> output = switch (strategy) {
            case FILTER -> state.accepted;
            case LATEST_VALID -> latest(state.accepted);
            case WEIGHTED_AVERAGE -> weightedAverage(state.accepted);
            case COVARIANCE_INTERSECTION -> covarianceIntersection(state.accepted);
            case KALMAN -> kalman(odometry, state.accepted, timestampSeconds);
        };
        state.output = CycleResult.of(output);
    }

    private static List<Measurement> rejectedMeasurements(List<Rejection> rejections) {
        Measurement[] measurements = new Measurement[rejections.size()];
        for (int index = 0; index < measurements.length; index++) {
            Measurement measurement = rejections.get(index).measurement();
            if (measurement == null) {
                // Unmodifiable like List.of, but keeps a null measurement.
                return rejections.stream().map(Rejection::measurement).toList();
            }
            measurements[index] = measurement;
        }
        return List.of(measurements);
    }

    private static <T> List<T> added(List<T> list, T value) {
        List<T> target = list == null ? new ArrayList<>() : list;
        target.add(value);
        return target;
    }

    private void publishPose() {
        if (!publishNetworkTables) {
            return;
        }
        PoseSnapshot latest = state.output.latestPose() == null ? null : state.output.latestPose().pose();
        if (latest == null) {
            return;
        }
        if (state.posePublisher == null) {
            String topic = debugName.isBlank()
                    ? "Athena/Localization/Pose"
                    : "Athena/Localization/" + debugName + "/Pose";
            state.posePublisher = NetworkTableInstance.getDefault()
                    .getStructTopic(topic, Pose2d.struct)
                    .publish();
        }
        state.posePublisher.set(toWpilib(latest));
    }

    private List<Measurement> kalman(
            SwerveOdometry odometry,
            List<Measurement> vision,
            double timestampSeconds) {
        if (odometry == null) {
            throw new IllegalStateException("Kalman localization requires one swerve odometry PoseSignal input.");
        }
        if (!Double.isFinite(timestampSeconds)) {
            return state.output.measurements();
        }
        if (state.kalman == null) {
            state.kalman = new KalmanEngine(odometry, stateStdDevs, defaultVisionStdDevs);
        }
        observeFrames(timestampSeconds);
        boolean seeding = tuning.seedWhileDisabled() != null && tuning.seedWhileDisabled().getAsBoolean();
        PoseSnapshot estimated = state.kalman.update(
                odometry,
                vision,
                timestampSeconds,
                innovationGate,
                maxHeadingDisagreementRadians,
                state.pendingReset,
                tuning,
                seeding,
                state.vision);
        state.pendingReset = null;
        publishVisionStatus(timestampSeconds);
        return List.of(outputMeasurement(
                estimated,
                odometry.velocity(),
                timestampSeconds,
                stateStdDevs,
                this));
    }

    /**
     * Records this cycle's frame verdicts: the node's own rejections, and the rejections of every filter stage
     * upstream of it. Accepted frames are recorded by the Kalman engine when it judges them.
     */
    private void observeFrames(double nowSeconds) {
        for (Rejection rejection : state.rejections) {
            observeRejection(rejection, nowSeconds);
        }
        for (Localization stage : upstreamStages()) {
            List<Rejection> upstream;
            synchronized (stage.state) {
                upstream = stage.state.rejections;
            }
            for (Rejection rejection : upstream) {
                observeRejection(rejection, nowSeconds);
            }
        }
    }

    private void observeRejection(Rejection rejection, double nowSeconds) {
        if (rejection.measurement() instanceof PoseMeasurementSample sample) {
            state.vision.observe(sample.source(), sample.timestampSeconds(),
                    nowSeconds, Verdict.reject(rejection.reason()), sample.targetCount());
        }
    }

    /** Filter stages feeding this node, found through its refreshable inputs, nearest first. */
    private List<Localization> upstreamStages() {
        // The graph is fixed once the refreshable inputs are cached, so the stage list is computed once.
        List<Localization> cached = state.stages;
        if (cached != null) {
            return cached;
        }
        List<Localization> stages = new ArrayList<>();
        collectStages(this, stages, java.util.Collections.newSetFromMap(new IdentityHashMap<>()));
        state.stages = List.copyOf(stages);
        return state.stages;
    }

    private static void collectStages(Localization node, List<Localization> stages, Set<Localization> visited) {
        for (MeasurementSignal upstream : node.refreshableUpstreams()) {
            if (upstream instanceof Localization child && visited.add(child)) {
                stages.add(child);
                collectStages(child, stages, visited);
            }
        }
    }

    private void publishVisionStatus(double nowSeconds) {
        state.visionNow = nowSeconds;
        List<VisionSourceStatus> statuses = state.vision.snapshot(nowSeconds);
        VisionStatusRegistry.publish(state, statuses);
        if (publishNetworkTables) {
            for (VisionSourceStatus status : statuses) {
                state.visionTopics.computeIfAbsent(status.name(), VisionTopics::new).publish(status);
            }
        }
    }

    private List<Measurement> latest(List<Measurement> measurements) {
        Measurement latest = null;
        for (Measurement measurement : measurements) {
            if (latest == null
                    || Double.compare(measurement.timestampSeconds(), latest.timestampSeconds()) > 0) {
                latest = measurement;
            }
        }
        return latest == null ? List.of() : List.of(latest);
    }

    private List<Measurement> weightedAverage(List<Measurement> measurements) {
        List<PoseMeasurementSample> samples = poseSamples(measurements);
        if (samples.isEmpty()) {
            return List.of();
        }
        double x = 0.0;
        double y = 0.0;
        double sin = 0.0;
        double cos = 0.0;
        double vx = 0.0;
        double vy = 0.0;
        double omega = 0.0;
        double totalWeight = 0.0;
        double timestamp = 0.0;
        double ambiguity = 0.0;
        double distance = 0.0;
        double distanceWeight = 0.0;
        int targetCount = 0;
        for (PoseMeasurementSample sample : samples) {
            double variance = finiteVariance(sample.stdDevs().raw().translationVarianceMetersSquared(), 1.0);
            double weight = 1.0 / variance;
            x += sample.pose().xMeters() * weight;
            y += sample.pose().yMeters() * weight;
            sin += Math.sin(sample.pose().headingRadians()) * weight;
            cos += Math.cos(sample.pose().headingRadians()) * weight;
            vx += sample.speeds().raw().xMetersPerSecond() * weight;
            vy += sample.speeds().raw().yMetersPerSecond() * weight;
            omega += sample.speeds().raw().angularRadiansPerSecond() * weight;
            totalWeight += weight;
            timestamp = Math.max(timestamp, sample.timestampSeconds());
            ambiguity += sample.ambiguity() * weight;
            targetCount += sample.targetCount();
            if (Double.isFinite(sample.averageTargetDistanceMeters())) {
                distance += sample.averageTargetDistanceMeters() * weight;
                distanceWeight += weight;
            }
        }
        MeasurementStdDevs outputStdDevs = MeasurementStdDevs.of(
                Meters.of(Math.sqrt(1.0 / totalWeight)),
                Meters.of(Math.sqrt(1.0 / totalWeight)),
                Radians.of(samples.stream().map(PoseMeasurementSample::stdDevs)
                        .mapToDouble(stdDevs -> stdDevs.raw().headingRadians()).min().orElse(1.0)));
        return List.of(outputMeasurement(
                new PoseSnapshot(x / totalWeight, y / totalWeight, Math.atan2(sin, cos)),
                RobotVelocity.Raw.of(vx / totalWeight, vy / totalWeight, omega / totalWeight, VelocityFrame.ROBOT),
                timestamp,
                outputStdDevs,
                this,
                ambiguity / totalWeight,
                targetCount,
                distanceWeight > 0.0 ? distance / distanceWeight : Double.NaN));
    }

    private List<Measurement> covarianceIntersection(List<Measurement> measurements) {
        List<PoseMeasurementSample> samples = groupedSamples(measurements);
        if (samples.isEmpty()) {
            return List.of();
        }
        FusedEstimate fused = FusedEstimate.from(samples.get(0), defaultVisionStdDevs);
        for (int index = 1; index < samples.size(); index++) {
            fused = fused.intersect(FusedEstimate.from(samples.get(index), defaultVisionStdDevs));
        }
        return List.of(outputMeasurement(
                fused.pose,
                fused.speeds,
                fused.timestampSeconds,
                fused.stdDevs(),
                this,
                fused.ambiguity,
                fused.targetCount,
                fused.averageTargetDistanceMeters));
    }

    private List<PoseMeasurementSample> groupedSamples(List<Measurement> measurements) {
        double newestTimestamp = Double.NEGATIVE_INFINITY;
        boolean foundSample = false;
        for (Measurement measurement : measurements) {
            if (measurement instanceof PoseMeasurementSample sample) {
                foundSample = true;
                newestTimestamp = Math.max(newestTimestamp, sample.timestampSeconds());
            }
        }
        if (!foundSample) {
            return List.of();
        }

        // One sample per source: frames from the same camera are not independent estimates, and a
        // high-frame-rate camera can deliver several per cycle. Keeping only its newest frame stops it from
        // outweighing other cameras and from counting its tags several times.
        Map<Object, PoseMeasurementSample> newestBySource = null;
        List<PoseMeasurementSample> recent = new ArrayList<>();
        for (Measurement measurement : measurements) {
            if (measurement instanceof PoseMeasurementSample sample
                    && newestTimestamp - sample.timestampSeconds() <= groupWindowSeconds) {
                if (sample.source() == null) {
                    recent.add(sample);
                    continue;
                }
                if (newestBySource == null) {
                    newestBySource = new IdentityHashMap<>();
                }
                PoseMeasurementSample previous = newestBySource.get(sample.source());
                if (previous == null || sample.timestampSeconds() > previous.timestampSeconds()) {
                    newestBySource.put(sample.source(), sample);
                }
            }
        }
        if (newestBySource != null) {
            for (Measurement measurement : measurements) {
                if (measurement instanceof PoseMeasurementSample sample
                        && sample.source() != null
                        && newestBySource.get(sample.source()) == sample) {
                    recent.add(sample);
                }
            }
        }

        boolean[] candidateMembers = new boolean[recent.size()];
        boolean[] bestMembers = new boolean[recent.size()];
        ClusterStats candidate = new ClusterStats();
        ClusterStats best = new ClusterStats();
        Set<Object> sources = new HashSet<>();
        for (PoseMeasurementSample anchor : recent) {
            candidate.clear();
            sources.clear();
            for (int index = 0; index < recent.size(); index++) {
                PoseMeasurementSample sample = recent.get(index);
                boolean member = agrees(anchor, sample);
                candidateMembers[index] = member;
                if (member) {
                    candidate.add(sample, sources);
                }
            }
            if (candidate.betterThan(best)) {
                System.arraycopy(candidateMembers, 0, bestMembers, 0, candidateMembers.length);
                best.copyFrom(candidate);
            }
        }
        if (best.sourceCount < minimumSourceCount) {
            return List.of();
        }
        List<PoseMeasurementSample> result = new ArrayList<>(best.size);
        for (int index = 0; index < recent.size(); index++) {
            if (bestMembers[index]) {
                result.add(recent.get(index));
            }
        }
        return result;
    }

    private boolean agrees(PoseMeasurementSample first, PoseMeasurementSample second) {
        if (trustsTranslation(first)
                && trustsTranslation(second)
                && translationDistance(first.pose(), second.pose()) > maxTranslationDisagreementMeters) {
            return false;
        }
        if (!trustsHeading(first) || !trustsHeading(second)) {
            return true;
        }
        return Math.abs(wrapRadians(first.pose().headingRadians() - second.pose().headingRadians()))
                <= maxHeadingDisagreementRadians;
    }

    private static boolean trustsHeading(PoseMeasurementSample sample) {
        return sample.stdDevs() == null || sample.stdDevs().raw().headingRadians() <= Math.PI;
    }

    private static boolean trustsTranslation(PoseMeasurementSample sample) {
        return sample.stdDevs() == null
                || (sample.stdDevs().raw().xMeters() < 1.0e6 && sample.stdDevs().raw().yMeters() < 1.0e6);
    }

    /** Null when every filter accepts the sample, otherwise the reason of the first filter that rejects it. */
    private String rejection(Measurement measurement, PoseSnapshot pose) {
        for (int index = 0; index < filters.size(); index++) {
            LocalizationFilter filter = filters.get(index);
            if (filter instanceof ReasonedFilter reasoned) {
                String reason = reasoned.rejection(this, measurement, pose);
                if (reason != null) {
                    return reason;
                }
            } else if (!filter.accept(this, measurement, pose)) {
                String reason = filter.rejectionReason(this, measurement, pose);
                if (reason == null || reason.isBlank()) {
                    String name = filter.name();
                    reason = "filter " + (name == null || name.isBlank() ? "#" + (index + 1) : name);
                }
                return reason;
            }
        }
        return null;
    }

    private void resetInputs(PoseSnapshot pose) {
        for (PoseSignal input : inputs) {
            if (input instanceof ResettablePoseSignal resettable) {
                resettable.reset(pose);
            } else if (input instanceof Localization localization) {
                localization.resetInputs(pose);
            }
        }
    }

    private Localization copy(List<PoseSignal> nextInputs, List<LocalizationFilter> nextFilters) {
        return new Localization(strategy, nextInputs, nextFilters, debugName, publishNetworkTables,
                groupWindowSeconds, maxTranslationDisagreementMeters, maxHeadingDisagreementRadians,
                minimumSourceCount, innovationGate, stateStdDevs, defaultVisionStdDevs, tuning, new State());
    }

    private Localization configured(
            String name,
            boolean publish,
            double groupWindow,
            double translationDisagreement,
            double headingDisagreement,
            double gate,
            MeasurementStdDevs processStdDevs,
            MeasurementStdDevs visionStdDevs) {
        return new Localization(strategy, inputs, filters, name, publish, groupWindow,
                translationDisagreement, headingDisagreement, minimumSourceCount,
                gate, processStdDevs, visionStdDevs, tuning, new State());
    }

    private static PoseSnapshot pose(Measurement measurement) {
        return measurement instanceof PoseMeasurementSample sample ? sample.pose() : null;
    }

    private static List<PoseMeasurementSample> poseSamples(List<Measurement> measurements) {
        return measurements.stream()
                .filter(PoseMeasurementSample.class::isInstance)
                .map(PoseMeasurementSample.class::cast)
                .toList();
    }

    private static Measurement outputMeasurement(
            PoseSnapshot pose,
            RobotVelocity speeds,
            double timestampSeconds,
            MeasurementStdDevs stdDevs,
            Object source) {
        return Measurements.poseAndSpeeds(pose, speeds)
                .timing(Seconds.of(timestampSeconds), Seconds.of(0.0))
                .stdDevs(stdDevs)
                .source(source);
    }

    private static Measurement outputMeasurement(
            PoseSnapshot pose,
            RobotVelocity speeds,
            double timestampSeconds,
            MeasurementStdDevs stdDevs,
            Object source,
            double ambiguity,
            int targetCount,
            double averageTargetDistanceMeters) {
        return Measurements.poseAndSpeeds(pose, speeds)
                .timing(Seconds.of(timestampSeconds), Seconds.of(0.0))
                .visionMetadata(ambiguity, targetCount, averageTargetDistanceMeters)
                .stdDevs(stdDevs)
                .source(source);
    }

    private static double finiteVariance(double variance, double fallback) {
        return Double.isFinite(variance) && variance > 1.0e-12 ? variance : fallback;
    }

    private static double translationDistance(PoseSnapshot first, PoseSnapshot second) {
        return Math.hypot(first.xMeters() - second.xMeters(), first.yMeters() - second.yMeters());
    }

    private static double wrapRadians(double radians) {
        return radians - Math.floor((radians + Math.PI) / TWO_PI) * TWO_PI;
    }

    private static double nonNegative(double value, String name) {
        if (Double.isNaN(value) || value < 0.0) {
            throw new IllegalArgumentException(name + " must be non-negative.");
        }
        return value;
    }

    /** A measurement a filter stage rejected, with the reason that names the rejecting filter. */
    private record Rejection(Measurement measurement, String reason) {
    }

    /** Kalman-only settings carried by the builder. {@link #NONE} is the default. */
    private record KalmanTuning(
            double speedScaleLinear,
            double speedScaleAngular,
            double maxAngularVelocity,
            BooleanSupplier seedWhileDisabled,
            BooleanSupplier visionHeadingWhile) {
        static final KalmanTuning NONE = new KalmanTuning(0.0, 0.0, Double.POSITIVE_INFINITY, null, null);
    }

    /** NetworkTables publishers for one camera's status, created on first sight. */
    private static final class VisionTopics {
        private final StringPublisher state;
        private final StringPublisher reason;
        private final DoublePublisher acceptedPerSecond;
        private final DoublePublisher rejectedPerSecond;
        private final DoublePublisher frameAgeMs;
        private final IntegerPublisher tagCount;
        private final DoublePublisher secondsSinceAccepted;
        private final String camera;
        private edu.wpi.first.networktables.BooleanPublisher timeSync;

        private VisionTopics(String camera) {
            this.camera = camera;
            NetworkTableInstance nt = NetworkTableInstance.getDefault();
            state = nt.getStringTopic(VisionSourceStatus.path(camera, "State")).publish();
            reason = nt.getStringTopic(VisionSourceStatus.path(camera, "Reason")).publish();
            acceptedPerSecond = nt.getDoubleTopic(VisionSourceStatus.path(camera, "AcceptedPerSecond")).publish();
            rejectedPerSecond = nt.getDoubleTopic(VisionSourceStatus.path(camera, "RejectedPerSecond")).publish();
            frameAgeMs = nt.getDoubleTopic(VisionSourceStatus.path(camera, "FrameAgeMs")).publish();
            tagCount = nt.getIntegerTopic(VisionSourceStatus.path(camera, "TagCount")).publish();
            secondsSinceAccepted = nt.getDoubleTopic(VisionSourceStatus.path(camera, "SecondsSinceAccepted")).publish();
        }

        private void publish(VisionSourceStatus status) {
            state.set(status.state().name());
            reason.set(status.reason());
            acceptedPerSecond.set(status.acceptedRate().in(Hertz));
            rejectedPerSecond.set(status.rejectedRate().in(Hertz));
            if (Double.isFinite(status.frameAge().in(Seconds))) {
                frameAgeMs.set(status.frameAge().in(Milliseconds));
            }
            tagCount.set(status.tagCount());
            if (Double.isFinite(status.sinceAccepted().in(Seconds))) {
                secondsSinceAccepted.set(status.sinceAccepted().in(Seconds));
            }
            // Only cameras that can tell whether their clock is synced get a TimeSync topic.
            if (status.timeSync() != null) {
                if (timeSync == null) {
                    timeSync = NetworkTableInstance.getDefault()
                            .getBooleanTopic(VisionSourceStatus.path(camera, "TimeSync")).publish();
                }
                timeSync.set(status.timeSync());
            }
        }
    }

    private static final class State {
        private CycleResult output = CycleResult.empty();
        private List<Measurement> accepted = List.of();
        private List<Measurement> rejected = List.of();
        private List<Rejection> rejections = List.of();
        private final VisionDiagnostics vision = new VisionDiagnostics();
        private double visionNow = Double.NaN;
        private final Map<String, VisionTopics> visionTopics = new HashMap<>();
        private PoseSnapshot pendingReset;
        private boolean seeded;
        private double lastRefreshTimestamp = Double.NaN;
        private boolean runtimeOwned;
        private KalmanEngine kalman;
        private StructPublisher<Pose2d> posePublisher;
        private List<MeasurementSignal> refreshables;
        private List<RobotHeadingAware> headingCameras;
        private List<Localization> stages;
        private double evaluatingTimestamp = Double.NaN;
    }

    private static final class CycleResult implements PoseSignal, MeasurementSnapshot.CycleView {
        private static final CycleResult EMPTY = new CycleResult(List.of(), null, null);
        private final List<Measurement> measurements;
        private final Measurement latest;
        private final PoseMeasurementSample latestPose;

        private CycleResult(
                List<Measurement> measurements,
                Measurement latest,
                PoseMeasurementSample latestPose) {
            this.measurements = measurements;
            this.latest = latest;
            this.latestPose = latestPose;
        }

        private static CycleResult empty() {
            return EMPTY;
        }

        private static CycleResult of(List<Measurement> values) {
            if (values.isEmpty()) {
                return EMPTY;
            }
            Measurement latest = null;
            PoseMeasurementSample latestPose = null;
            for (Measurement measurement : values) {
                if (latest == null
                        || Double.compare(measurement.timestampSeconds(), latest.timestampSeconds()) > 0) {
                    latest = measurement;
                }
                if (measurement instanceof PoseMeasurementSample pose
                        && (latestPose == null
                                || Double.compare(pose.timestampSeconds(), latestPose.timestampSeconds()) > 0)) {
                    latestPose = pose;
                }
            }
            // Every strategy creates an immutable list before publishing the cycle result.
            return new CycleResult(values, latest, latestPose);
        }

        @Override
        public List<Measurement> measurements() {
            return measurements;
        }

        @Override
        public Optional<Measurement> latestMeasurement() {
            return Optional.ofNullable(latest);
        }

        private PoseMeasurementSample latestPose() {
            return latestPose;
        }
    }

    private static final class ClusterStats {
        private int sourceCount;
        private int size;
        private int targetCount;
        private double ambiguity;
        private double newestTimestamp;

        private void clear() {
            sourceCount = 0;
            size = 0;
            targetCount = 0;
            ambiguity = 0.0;
            newestTimestamp = 0.0;
        }

        private void add(PoseMeasurementSample sample, Set<Object> sources) {
            Object source = sample.source() == null ? sample : sample.source();
            if (sources.add(source)) {
                sourceCount++;
            }
            newestTimestamp = size == 0
                    ? sample.timestampSeconds()
                    : Math.max(newestTimestamp, sample.timestampSeconds());
            size++;
            targetCount += sample.targetCount();
            ambiguity += sample.ambiguity();
        }

        private boolean betterThan(ClusterStats current) {
            if (sourceCount != current.sourceCount) {
                return sourceCount > current.sourceCount;
            }
            if (size != current.size) {
                return size > current.size;
            }
            if (targetCount != current.targetCount) {
                return targetCount > current.targetCount;
            }
            if (Double.compare(ambiguity, current.ambiguity) != 0) {
                return ambiguity < current.ambiguity;
            }
            return newestTimestamp > current.newestTimestamp;
        }

        private void copyFrom(ClusterStats source) {
            sourceCount = source.sourceCount;
            size = source.size;
            targetCount = source.targetCount;
            ambiguity = source.ambiguity;
            newestTimestamp = source.newestTimestamp;
        }
    }

    private static final class KalmanEngine {
        private static final Object UNKNOWN_VISION_SOURCE = new Object();
        private static final int RECOVERY_SAMPLE_COUNT = 3;
        /** Clock skew allowed before a vision frame counts as coming from the future. */
        private static final double FUTURE_TOLERANCE_SECONDS = 0.1;
        private static final int SINGLE_TAG_RECOVERY_SAMPLE_COUNT = 5;
        private static final double RECOVERY_MAX_INTERVAL_SECONDS = 0.25;
        private static final double RECOVERY_TRANSLATION_METERS = 0.75;
        private static final double RECOVERY_HEADING_RADIANS = Math.toRadians(25.0);
        /** Minimum robot time between two multi-tag seeds while disabled. */
        private static final double SEED_INTERVAL_SECONDS = 0.25;
        /** Consistent frames needed to seed an unknown pose from vision: multi-tag, then single-tag. */
        private static final int SEED_SAMPLE_COUNT = 2;
        private static final int SINGLE_TAG_SEED_SAMPLE_COUNT = 4;
        private static final double SEED_MAX_INTERVAL_SECONDS = 0.25;
        private static final double SEED_TRANSLATION_METERS = 0.3;
        private static final double SEED_HEADING_RADIANS = Math.toRadians(10.0);
        /** Heading std dev of a frame whose heading is ignored; matches PoseSignal.translationOnly(). */
        private static final double UNTRUSTED_HEADING_STD_DEV_RADIANS = 1.0e6;
        private final SwerveDriveKinematics kinematics;
        private final MeasurementStdDevs stateStdDevs;
        private final MeasurementStdDevs defaultVisionStdDevs;
        private final Map<Object, Double> lastSeenVisionTimestamps = new IdentityHashMap<>();
        private final Map<Object, VisionRecovery> visionRecovery = new IdentityHashMap<>();
        private final Map<Object, SeedCandidate> seedCandidates = new IdentityHashMap<>();
        /** Whether the field translation and heading are known, from a reset or from vision. */
        private boolean translationSeeded;
        private boolean headingSeeded;
        private SwerveDrivePoseEstimator estimator;
        /** Last vision std-dev vector handed to the estimator; reused while the three values are unchanged. */
        private edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> visionVector;
        private final long[] visionBits = new long[3];
        private double lastSeedTimestamp = Double.NEGATIVE_INFINITY;
        private final double[] moduleDistances;
        private final double[] moduleAngles;

        private KalmanEngine(
                SwerveOdometry odometry,
                MeasurementStdDevs stateStdDevs,
                MeasurementStdDevs defaultVisionStdDevs) {
            this.stateStdDevs = stateStdDevs;
            this.defaultVisionStdDevs = defaultVisionStdDevs;
            moduleDistances = new double[odometry.kinematics().modules().size()];
            moduleAngles = new double[moduleDistances.length];
            kinematics = new SwerveDriveKinematics(odometry.kinematics().modules().stream()
                    .map(module -> new Translation2d(module.x().in(edu.wpi.first.units.Units.Meters), module.y().in(edu.wpi.first.units.Units.Meters)))
                    .toArray(Translation2d[]::new));
        }

        private PoseSnapshot update(
                SwerveOdometry odometry,
                List<Measurement> vision,
                double timestampSeconds,
                double innovationGate,
                double maxHeadingResidualRadians,
                PoseSnapshot reset,
                KalmanTuning tuning,
                boolean seeding,
                VisionDiagnostics diagnostics) {
            // Same value as odometry.heading() in radians, without the measure.
            Rotation2d heading = new Rotation2d(odometry.poseSnapshot().headingRadians());
            SwerveModulePosition[] positions = positions(odometry);
            RobotVelocity odometrySpeeds = odometry.velocity();
            if (reset != null) {
                translationSeeded = true;
                headingSeeded = true;
                seedCandidates.clear();
            }
            if (estimator == null) {
                PoseSnapshot initial = reset == null ? odometry.poseSnapshot() : reset;
                estimator = new SwerveDrivePoseEstimator(
                        kinematics,
                        heading,
                        positions,
                        toWpilib(initial),
                        vector(stateStdDevs),
                        vector(defaultVisionStdDevs));
            } else if (reset != null) {
                estimator.resetPosition(heading, positions, toWpilib(reset));
                lastSeenVisionTimestamps.clear();
                visionRecovery.clear();
            }
            estimator.updateWithTime(timestampSeconds, heading, positions);
            for (PoseMeasurementSample sample : oldestFirst(vision)) {
                Verdict verdict = addVision(sample, timestampSeconds, innovationGate, maxHeadingResidualRadians,
                        odometrySpeeds, heading, positions, tuning, seeding);
                diagnostics.observe(sample.source(), sample.timestampSeconds(),
                        timestampSeconds, verdict, sample.targetCount());
            }
            return fromWpilib(estimator.getEstimatedPosition());
        }

        private Verdict addVision(
                PoseMeasurementSample sample,
                double nowSeconds,
                double innovationGate,
                double maxHeadingResidualRadians,
                RobotVelocity odometrySpeeds,
                Rotation2d heading,
                SwerveModulePosition[] positions,
                KalmanTuning tuning,
                boolean seeding) {
            // A frame from the future means the camera clock is not synced to the robot yet (PhotonVision
            // and Limelight both report robot time only after time sync). Ignore it without remembering
            // its timestamp: one such frame would otherwise make every later, correct frame look old and
            // lock the camera out until the next reset.
            if (Double.isFinite(nowSeconds) && sample.timestampSeconds() > nowSeconds + FUTURE_TOLERANCE_SECONDS) {
                return Verdict.reject(LocalizationFilters.format("frame %.1f s in the future",
                        sample.timestampSeconds() - nowSeconds));
            }
            Object source = sample.source() == null ? UNKNOWN_VISION_SOURCE : sample.source();
            double previous = lastSeenVisionTimestamps.getOrDefault(source, Double.NEGATIVE_INFINITY);
            if (sample.timestampSeconds() <= previous) {
                return Verdict.reject("older than the last frame from this camera");
            }
            lastSeenVisionTimestamps.put(source, sample.timestampSeconds());
            RobotVelocity speeds = measuredSpeeds(sample, odometrySpeeds);
            double spin = Math.abs(speeds.raw().angularRadiansPerSecond());
            if (spin > tuning.maxAngularVelocity()) {
                return Verdict.reject(LocalizationFilters.format("spinning %.1f rad/s > %.1f",
                        spin, tuning.maxAngularVelocity()));
            }
            boolean visionHeading = tuning.visionHeadingWhile() == null || tuning.visionHeadingWhile().getAsBoolean();
            if (!visionHeading && headingSeeded && trustsHeading(sample)) {
                if (!trustsTranslation(sample)) {
                    return Verdict.reject("camera heading not used while enabled");
                }
                MeasurementStdDevs trusted = usable(sample.stdDevs()) ? sample.stdDevs() : defaultVisionStdDevs;
                sample = new TranslationOnlySample(sample, MeasurementStdDevs.of(
                        Meters.of(trusted.raw().xMeters()),
                        Meters.of(trusted.raw().yMeters()),
                        Radians.of(UNTRUSTED_HEADING_STD_DEV_RADIANS)));
            }
            if (seeding && sample.targetCount() >= 2
                    && sample.timestampSeconds() - lastSeedTimestamp >= SEED_INTERVAL_SECONDS) {
                // Disabled and a multi-tag frame: take the vision pose outright, the parts the frame trusts.
                seedFrom(sample, trustsTranslation(sample), trustsHeading(sample), heading, positions);
                lastSeedTimestamp = sample.timestampSeconds();
                visionRecovery.remove(source);
                return Verdict.accept();
            }
            if (seedUnknownPose(source, sample, heading, positions)) {
                return Verdict.accept();
            }
            MeasurementStdDevs base = usable(sample.stdDevs()) ? sample.stdDevs() : defaultVisionStdDevs;
            double scale = 1.0
                    + tuning.speedScaleLinear() * Math.hypot(speeds.raw().xMetersPerSecond(), speeds.raw().yMetersPerSecond())
                    + tuning.speedScaleAngular() * spin;
            MeasurementStdDevs stdDevs = MeasurementStdDevs.of(
                    Meters.of(base.raw().xMeters() * scale), Meters.of(base.raw().yMeters() * scale), Radians.of(base.raw().headingRadians() * scale));
            // Heading-residual and innovation gates share one recovery path: a source whose samples
            // are rejected but stay self-consistent for several frames (e.g. after a gyro slip or a
            // reset to the wrong heading) is re-admitted instead of being locked out forever.
            // The filter state does not change between the two checks, so one prediction serves both.
            Pose2d predicted = estimator.sampleAt(sample.timestampSeconds()).orElse(estimator.getEstimatedPosition());
            double headingResidual = headingResidual(sample, predicted);
            boolean headingRejected = trustsHeading(sample) && headingResidual > maxHeadingResidualRadians;
            double innovationValue = Double.isFinite(innovationGate) ? innovation(sample, stdDevs, predicted) : 0.0;
            boolean innovationRejected = innovationValue > innovationGate;
            if (headingRejected || innovationRejected) {
                String reason = headingRejected
                        ? LocalizationFilters.format("heading off %.1f deg > %.1f deg",
                                Math.toDegrees(headingResidual), Math.toDegrees(maxHeadingResidualRadians))
                        : LocalizationFilters.format("innovation %.1f > %.1f", innovationValue, innovationGate);
                VisionRecovery recovery = visionRecovery.computeIfAbsent(source, ignored -> new VisionRecovery());
                if (!recovery.accepts(sample)) {
                    return Verdict.recovering(LocalizationFilters.format("%s (recovering %d/%d)",
                            reason, recovery.consistentSamples, recovery.requiredSamples));
                }
            } else {
                visionRecovery.remove(source);
            }
            estimator.addVisionMeasurement(toWpilib(sample.pose()), sample.timestampSeconds(), visionVector(stdDevs));
            return Verdict.accept();
        }

        /**
         * Moves the estimate straight to vision while its field pose is not known yet. The estimate starts at the
         * odometry pose, usually the origin, and claims to be accurate there; without seeding, the innovation gate
         * treats the real pose as an outlier and each accepted frame only blends a few percent of the way, so a
         * robot that boots seeing one tag can take a long time to localize. Seeding needs a few self-consistent
         * frames from one source; a frame that does not complete a seed takes the normal gated path.
         *
         * @return true when the frame seeded the estimate
         */
        private boolean seedUnknownPose(
                Object source,
                PoseMeasurementSample sample,
                Rotation2d heading,
                SwerveModulePosition[] positions) {
            boolean seedTranslation = !translationSeeded && trustsTranslation(sample);
            boolean seedHeading = !headingSeeded && trustsHeading(sample);
            if (!seedTranslation && !seedHeading) {
                seedCandidates.remove(source);
                return false;
            }
            SeedCandidate candidate = seedCandidates.computeIfAbsent(source, ignored -> new SeedCandidate());
            if (!candidate.accepts(sample)) {
                return false;
            }
            seedCandidates.clear();
            seedFrom(sample, seedTranslation, seedHeading, heading, positions);
            return true;
        }

        /** Resets the estimate to the chosen parts of a vision frame, carrying the motion since its capture. */
        private void seedFrom(
                PoseMeasurementSample sample,
                boolean seedTranslation,
                boolean seedHeading,
                Rotation2d heading,
                SwerveModulePosition[] positions) {
            Pose2d current = estimator.getEstimatedPosition();
            Pose2d atCapture = estimator.sampleAt(sample.timestampSeconds()).orElse(current);
            Pose2d visionAtCapture = new Pose2d(
                    seedTranslation ? sample.pose().xMeters() : atCapture.getX(),
                    seedTranslation ? sample.pose().yMeters() : atCapture.getY(),
                    seedHeading ? new Rotation2d(sample.pose().headingRadians()) : atCapture.getRotation());
            estimator.resetPosition(heading, positions, visionAtCapture.plus(current.minus(atCapture)));
            translationSeeded |= seedTranslation;
            headingSeeded |= seedHeading;
            visionRecovery.clear();
        }

        /** A frame whose heading is ignored: the gyro owns heading while camera heading is not wanted. */
        private record TranslationOnlySample(PoseMeasurementSample delegate, MeasurementStdDevs stdDevs)
                implements PoseMeasurementSample {
            @Override public PoseSnapshot pose() { return delegate.pose(); }
            @Override public RobotVelocity speeds() { return delegate.speeds(); }
            @Override public double timestampSeconds() { return delegate.timestampSeconds(); }
            @Override public double latencySeconds() { return delegate.latencySeconds(); }
            @Override public double ambiguity() { return delegate.ambiguity(); }
            @Override public int targetCount() { return delegate.targetCount(); }
            @Override public double averageTargetDistanceMeters() { return delegate.averageTargetDistanceMeters(); }
            @Override public Object source() { return delegate.source(); }
        }

        private static final class SeedCandidate {
            private PoseSnapshot previousPose;
            private double previousTimestamp = Double.NEGATIVE_INFINITY;
            private int consistentSamples;

            private boolean accepts(PoseMeasurementSample sample) {
                boolean continuous = previousPose != null
                        && sample.timestampSeconds() - previousTimestamp <= SEED_MAX_INTERVAL_SECONDS
                        && (!trustsTranslation(sample)
                                || translationDistance(previousPose, sample.pose()) <= SEED_TRANSLATION_METERS)
                        && (!trustsHeading(sample)
                                || Math.abs(wrapRadians(
                                        previousPose.headingRadians() - sample.pose().headingRadians()))
                                        <= SEED_HEADING_RADIANS);
                consistentSamples = continuous ? consistentSamples + 1 : 1;
                previousPose = sample.pose();
                previousTimestamp = sample.timestampSeconds();
                int required = sample.targetCount() >= 2 ? SEED_SAMPLE_COUNT : SINGLE_TAG_SEED_SAMPLE_COUNT;
                return consistentSamples >= required;
            }
        }

        /** Pose samples in timestamp order; equal timestamps keep their input order. */
        private static List<PoseMeasurementSample> oldestFirst(List<Measurement> vision) {
            int count = 0;
            for (Measurement measurement : vision) {
                if (measurement instanceof PoseMeasurementSample) {
                    count++;
                }
            }
            if (count == 0) {
                return List.of();
            }
            PoseMeasurementSample[] samples = new PoseMeasurementSample[count];
            int index = 0;
            for (Measurement measurement : vision) {
                if (measurement instanceof PoseMeasurementSample sample) {
                    samples[index++] = sample;
                }
            }
            if (count > 1) {
                java.util.Arrays.sort(samples, BY_TIMESTAMP);
            }
            return java.util.Arrays.asList(samples);
        }

        private static final Comparator<PoseMeasurementSample> BY_TIMESTAMP =
                Comparator.comparingDouble(PoseMeasurementSample::timestampSeconds);

        /** The frame's own speeds when it reports a non-zero set, otherwise the odometry speeds. */
        private static RobotVelocity measuredSpeeds(PoseMeasurementSample sample, RobotVelocity odometrySpeeds) {
            RobotVelocity speeds = sample.speeds();
            if (speeds != null
                    && Double.isFinite(speeds.raw().xMetersPerSecond())
                    && Double.isFinite(speeds.raw().yMetersPerSecond())
                    && Double.isFinite(speeds.raw().angularRadiansPerSecond())
                    && (speeds.raw().xMetersPerSecond() != 0.0
                            || speeds.raw().yMetersPerSecond() != 0.0
                            || speeds.raw().angularRadiansPerSecond() != 0.0)) {
                return speeds;
            }
            return odometrySpeeds == null ? RobotVelocity.zero() : odometrySpeeds;
        }

        private static final class VisionRecovery {
            private PoseSnapshot previousPose;
            private double previousTimestamp = Double.NEGATIVE_INFINITY;
            private int consistentSamples;
            private int requiredSamples;

            private boolean accepts(PoseMeasurementSample sample) {
                boolean continuous = previousPose != null
                        && sample.timestampSeconds() - previousTimestamp <= RECOVERY_MAX_INTERVAL_SECONDS
                        && (!trustsTranslation(sample)
                                || translationDistance(previousPose, sample.pose()) <= RECOVERY_TRANSLATION_METERS)
                        && (!trustsHeading(sample)
                                || Math.abs(wrapRadians(
                                        previousPose.headingRadians() - sample.pose().headingRadians()))
                                        <= RECOVERY_HEADING_RADIANS);
                consistentSamples = continuous ? consistentSamples + 1 : 1;
                previousPose = sample.pose();
                previousTimestamp = sample.timestampSeconds();
                requiredSamples = sample.targetCount() >= 2
                        ? RECOVERY_SAMPLE_COUNT
                        : SINGLE_TAG_RECOVERY_SAMPLE_COUNT;
                return consistentSamples >= requiredSamples;
            }
        }

        private double innovation(PoseMeasurementSample sample, MeasurementStdDevs stdDevs, Pose2d predicted) {
            double dx = sample.pose().xMeters() - predicted.getX();
            double dy = sample.pose().yMeters() - predicted.getY();
            double dh = wrapRadians(sample.pose().headingRadians() - predicted.getRotation().getRadians());
            return square(dx / stdDevs.raw().xMeters())
                    + square(dy / stdDevs.raw().yMeters())
                    + square(dh / stdDevs.raw().headingRadians());
        }

        private double headingResidual(PoseMeasurementSample sample, Pose2d predicted) {
            return Math.abs(wrapRadians(
                    sample.pose().headingRadians() - predicted.getRotation().getRadians()));
        }


        private SwerveModulePosition[] positions(SwerveOdometry odometry) {
            int count = odometry.modulePositions(moduleDistances, moduleAngles);
            SwerveModulePosition[] positions = new SwerveModulePosition[count];
            for (int index = 0; index < count; index++) {
                // The same values modulePositions() reports through its measures: meters are the base unit, and
                // an angle in rotations is read back through the base unit, as Angle.in(Rotations) does.
                positions[index] = new SwerveModulePosition(
                        moduleDistances[index],
                        Rotation2d.fromRotations(edu.wpi.first.units.Units.Rotations.of(moduleAngles[index])
                                .in(edu.wpi.first.units.Units.Rotations)));
            }
            return positions;
        }

        private edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> visionVector(
                MeasurementStdDevs stdDevs) {
            // Compared bit for bit so that -0.0 and NaN never reuse a vector built from different values.
            long x = Double.doubleToRawLongBits(stdDevs.raw().xMeters());
            long y = Double.doubleToRawLongBits(stdDevs.raw().yMeters());
            long heading = Double.doubleToRawLongBits(stdDevs.raw().headingRadians());
            if (visionVector == null || x != visionBits[0] || y != visionBits[1] || heading != visionBits[2]) {
                visionVector = vector(stdDevs);
                visionBits[0] = x;
                visionBits[1] = y;
                visionBits[2] = heading;
            }
            return visionVector;
        }

        private static boolean usable(MeasurementStdDevs stdDevs) {
            return stdDevs != null
                    && Double.isFinite(stdDevs.raw().xMeters())
                    && Double.isFinite(stdDevs.raw().yMeters())
                    && Double.isFinite(stdDevs.raw().headingRadians());
        }

        private static double square(double value) {
            return value * value;
        }
    }

    private record FusedEstimate(
            PoseSnapshot pose,
            RobotVelocity speeds,
            double timestampSeconds,
            double xVariance,
            double yVariance,
            double headingVariance,
            double ambiguity,
            int targetCount,
            double averageTargetDistanceMeters) {
        private static FusedEstimate from(PoseMeasurementSample sample, MeasurementStdDevs fallback) {
            MeasurementStdDevs stdDevs = KalmanEngine.usable(sample.stdDevs()) ? sample.stdDevs() : fallback;
            return new FusedEstimate(
                    sample.pose(),
                    sample.speeds(),
                    sample.timestampSeconds(),
                    square(stdDevs.raw().xMeters()),
                    square(stdDevs.raw().yMeters()),
                    square(stdDevs.raw().headingRadians()),
                    sample.ambiguity(),
                    sample.targetCount(),
                    sample.averageTargetDistanceMeters());
        }

        private FusedEstimate intersect(FusedEstimate other) {
            double bestWeight = 0.5;
            double bestTrace = Double.POSITIVE_INFINITY;
            for (int step = 0; step <= 100; step++) {
                double weight = step / 100.0;
                double trace = combinedVariance(xVariance, other.xVariance, weight)
                        + combinedVariance(yVariance, other.yVariance, weight)
                        + combinedVariance(headingVariance, other.headingVariance, weight);
                if (trace < bestTrace) {
                    bestTrace = trace;
                    bestWeight = weight;
                }
            }
            double otherHeading = pose.headingRadians()
                    + wrapRadians(other.pose.headingRadians() - pose.headingRadians());
            double xVar = combinedVariance(xVariance, other.xVariance, bestWeight);
            double yVar = combinedVariance(yVariance, other.yVariance, bestWeight);
            double hVar = combinedVariance(headingVariance, other.headingVariance, bestWeight);
            return new FusedEstimate(
                    new PoseSnapshot(
                            combinedMean(pose.xMeters(), xVariance, other.pose.xMeters(), other.xVariance, bestWeight),
                            combinedMean(pose.yMeters(), yVariance, other.pose.yMeters(), other.yVariance, bestWeight),
                            wrapRadians(combinedMean(
                                    pose.headingRadians(), headingVariance,
                                    otherHeading, other.headingVariance,
                                    bestWeight))),
                    RobotVelocity.Raw.of(
                            (speeds.raw().xMetersPerSecond() + other.speeds.raw().xMetersPerSecond()) / 2.0,
                            (speeds.raw().yMetersPerSecond() + other.speeds.raw().yMetersPerSecond()) / 2.0,
                            (speeds.raw().angularRadiansPerSecond() + other.speeds.raw().angularRadiansPerSecond()) / 2.0,
                            VelocityFrame.ROBOT),
                    fusedTimestamp(other, bestWeight),
                    xVar,
                    yVar,
                    hVar,
                    Math.max(ambiguity, other.ambiguity),
                    targetCount + other.targetCount,
                    averageFinite(averageTargetDistanceMeters, other.averageTargetDistanceMeters));
        }

        /**
         * Stamps the fused pose with the information-weighted mean of the operand timestamps.
         *
         * <p>Samples inside one group can be up to {@code groupWithinSeconds} apart and the pipeline
         * has no pose history here to motion-compensate them (camera samples carry no velocity and
         * odometry is only available inside the Kalman node). The fused mean pose is therefore a
         * blend of poses from different instants; stamping it with the same weights used to blend
         * the poses makes the timestamp describe the instant the pose actually represents, instead
         * of claiming the newest frame's time for data that is partly older. Keep the group window
         * small (default 30 ms) to bound the residual motion blur.</p>
         */
        private double fusedTimestamp(FusedEstimate other, double weight) {
            double first = weight * information();
            double second = (1.0 - weight) * other.information();
            double total = first + second;
            if (!Double.isFinite(total) || total <= 0.0) {
                return Math.max(timestampSeconds, other.timestampSeconds);
            }
            return (first * timestampSeconds + second * other.timestampSeconds) / total;
        }

        private double information() {
            return 1.0 / xVariance + 1.0 / yVariance + 1.0 / headingVariance;
        }

        private MeasurementStdDevs stdDevs() {
            return MeasurementStdDevs.of(
                    Meters.of(Math.sqrt(xVariance)),
                    Meters.of(Math.sqrt(yVariance)),
                    Radians.of(Math.sqrt(headingVariance)));
        }

        private static double combinedVariance(double first, double second, double weight) {
            return 1.0 / (weight / first + (1.0 - weight) / second);
        }

        private static double combinedMean(
                double firstMean,
                double firstVariance,
                double secondMean,
                double secondVariance,
                double weight) {
            double variance = combinedVariance(firstVariance, secondVariance, weight);
            return variance * (weight * firstMean / firstVariance
                    + (1.0 - weight) * secondMean / secondVariance);
        }

        private static double square(double value) {
            return value * value;
        }

        private static double averageFinite(double first, double second) {
            if (Double.isFinite(first) && Double.isFinite(second)) {
                return (first + second) / 2.0;
            }
            return Double.isFinite(first) ? first : second;
        }
    }

    private static edu.wpi.first.math.Matrix<edu.wpi.first.math.numbers.N3, edu.wpi.first.math.numbers.N1> vector(
            MeasurementStdDevs stdDevs) {
        return VecBuilder.fill(stdDevs.raw().xMeters(), stdDevs.raw().yMeters(), stdDevs.raw().headingRadians());
    }

    private static Pose2d toWpilib(PoseSnapshot pose) {
        return new Pose2d(pose.xMeters(), pose.yMeters(), new Rotation2d(pose.headingRadians()));
    }

    private static PoseSnapshot fromWpilib(Pose2d pose) {
        return new PoseSnapshot(pose.getX(), pose.getY(), pose.getRotation().getRadians());
    }
}
