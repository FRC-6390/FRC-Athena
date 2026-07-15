package ca.frc6390.athena.localization.pipeline;

import ca.frc6390.athena.drivetrain.swerve.SwerveOdometry;
import ca.frc6390.athena.hardware.runtime.ActionBinding;
import ca.frc6390.athena.hardware.runtime.ActionContext;
import ca.frc6390.athena.hardware.runtime.HardwareMeasurementSignal;
import ca.frc6390.athena.runtime.control.RobotVelocity;
import ca.frc6390.athena.runtime.filter.PoseSnapshot;
import ca.frc6390.athena.runtime.measurement.Measurement;
import ca.frc6390.athena.runtime.measurement.MeasurementStdDevs;
import ca.frc6390.athena.runtime.measurement.Measurements;
import ca.frc6390.athena.runtime.measurement.PoseMeasurementSample;
import ca.frc6390.athena.runtime.measurement.PoseSignal;
import ca.frc6390.athena.runtime.measurement.ResettablePoseSignal;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;

/** One inspectable pose-processing node in a localization graph. */
public final class Localization implements PoseSignal {
    enum Strategy {
        FILTER,
        LATEST_VALID,
        WEIGHTED_AVERAGE,
        COVARIANCE_INTERSECTION,
        KALMAN
    }

    private static final MeasurementStdDevs DEFAULT_STATE_STD_DEVS = MeasurementStdDevs.of(0.1, 0.1, 0.1);
    private static final MeasurementStdDevs DEFAULT_VISION_STD_DEVS = MeasurementStdDevs.of(0.9, 0.9, 0.9);
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

    public Localization groupWithinSeconds(double seconds) {
        return configured(debugName, publishNetworkTables, nonNegative(seconds, "Group window"),
                maxTranslationDisagreementMeters, maxHeadingDisagreementRadians, innovationGate,
                stateStdDevs, defaultVisionStdDevs);
    }

    public Localization maxTranslationDisagreementMeters(double meters) {
        return configured(debugName, publishNetworkTables, groupWindowSeconds,
                nonNegative(meters, "Translation disagreement"), maxHeadingDisagreementRadians,
                innovationGate, stateStdDevs, defaultVisionStdDevs);
    }

    public Localization maxHeadingDisagreementRadians(double radians) {
        return configured(debugName, publishNetworkTables, groupWindowSeconds,
                maxTranslationDisagreementMeters, nonNegative(radians, "Heading disagreement"),
                innovationGate, stateStdDevs, defaultVisionStdDevs);
    }

    /** Requires a fused camera group to contain at least this many distinct sources. */
    public Localization minimumSourceCount(int count) {
        if (count < 1) {
            throw new IllegalArgumentException("Minimum source count must be at least one.");
        }
        return new Localization(strategy, inputs, filters, debugName, publishNetworkTables,
                groupWindowSeconds, maxTranslationDisagreementMeters, maxHeadingDisagreementRadians,
                count, innovationGate, stateStdDevs, defaultVisionStdDevs, state);
    }

    /** Rejects vision whose squared residual divided by measurement variance exceeds this value. */
    public Localization maxNormalizedVisionResidual(double maximum) {
        return configured(debugName, publishNetworkTables, groupWindowSeconds,
                maxTranslationDisagreementMeters, maxHeadingDisagreementRadians,
                nonNegative(maximum, "Normalized vision residual"), stateStdDevs, defaultVisionStdDevs);
    }

    public Localization stateStdDevs(double translationMeters, double headingRadians) {
        return configured(debugName, publishNetworkTables, groupWindowSeconds,
                maxTranslationDisagreementMeters, maxHeadingDisagreementRadians, innovationGate,
                MeasurementStdDevs.of(translationMeters, translationMeters, headingRadians), defaultVisionStdDevs);
    }

    public Localization defaultVisionStdDevs(double translationMeters, double headingRadians) {
        return configured(debugName, publishNetworkTables, groupWindowSeconds,
                maxTranslationDisagreementMeters, maxHeadingDisagreementRadians, innovationGate,
                stateStdDevs, MeasurementStdDevs.of(translationMeters, translationMeters, headingRadians));
    }

    /** Refreshes this node and all of its inputs exactly once for the runtime timestamp. */
    public PoseSignal refresh(ActionContext context, double timestampSeconds, double dtSeconds) {
        synchronized (state) {
            if (Double.compare(state.lastRefreshTimestamp, timestampSeconds) == 0) {
                return snapshot(state.output);
            }
            refreshInputs(context, timestampSeconds, dtSeconds);
            evaluate(timestampSeconds);
            publishPose();
            state.lastRefreshTimestamp = timestampSeconds;
            state.runtimeOwned = true;
            return snapshot(state.output);
        }
    }

    @Override
    public List<Measurement> measurements() {
        synchronized (state) {
            if (!state.runtimeOwned && strategy != Strategy.KALMAN) {
                evaluate(Double.NaN);
            }
            return state.output;
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
        PoseSnapshot pose = pose();
        return new Pose2d(pose.xMeters(), pose.yMeters(), new Rotation2d(pose.headingRadians()));
    }

    public ActionBinding reset(PoseSnapshot pose) {
        Objects.requireNonNull(pose, "pose");
        return context -> {
            synchronized (state) {
                resetInputs(pose);
                state.pendingReset = pose;
                state.output = List.of(outputMeasurement(pose, RobotVelocity.zero(), 0.0,
                        stateStdDevs, this));
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

    public ActionBinding reset(double xMeters, double yMeters, double headingRadians) {
        return reset(new PoseSnapshot(xMeters, yMeters, headingRadians));
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

    private void refreshInputs(ActionContext context, double timestampSeconds, double dtSeconds) {
        for (PoseSignal input : inputs) {
            if (input instanceof Localization localization) {
                localization.refresh(context, timestampSeconds, dtSeconds);
            } else if (input instanceof HardwareMeasurementSignal hardware) {
                hardware.refresh(context, timestampSeconds, dtSeconds);
            }
        }
    }

    private void evaluate(double timestampSeconds) {
        List<Measurement> accepted = new ArrayList<>();
        List<Measurement> rejected = new ArrayList<>();
        SwerveOdometry odometry = null;
        for (PoseSignal input : inputs) {
            if (strategy == Strategy.KALMAN && input instanceof SwerveOdometry swerveOdometry) {
                if (odometry != null && odometry != swerveOdometry) {
                    throw new IllegalStateException("Kalman localization accepts exactly one odometry input.");
                }
                odometry = swerveOdometry;
                continue;
            }
            for (Measurement measurement : input.measurements()) {
                PoseSnapshot pose = pose(measurement);
                if (pose != null && accepts(measurement, pose)) {
                    accepted.add(measurement);
                } else {
                    rejected.add(measurement);
                }
            }
        }
        state.accepted = List.copyOf(accepted);
        state.rejected = List.copyOf(rejected);
        state.output = switch (strategy) {
            case FILTER -> List.copyOf(accepted);
            case LATEST_VALID -> latest(accepted);
            case WEIGHTED_AVERAGE -> weightedAverage(accepted);
            case COVARIANCE_INTERSECTION -> covarianceIntersection(accepted);
            case KALMAN -> kalman(odometry, accepted, timestampSeconds);
        };
    }

    private void publishPose() {
        if (!publishNetworkTables) {
            return;
        }
        PoseSnapshot latest = poseSamples(state.output).stream()
                .max(Comparator.comparingDouble(PoseMeasurementSample::timestampSeconds))
                .map(PoseMeasurementSample::pose)
                .orElse(null);
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
            return state.output;
        }
        if (state.kalman == null) {
            state.kalman = new KalmanEngine(odometry, stateStdDevs, defaultVisionStdDevs);
        }
        PoseSnapshot estimated = state.kalman.update(
                odometry,
                vision,
                timestampSeconds,
                innovationGate,
                maxHeadingDisagreementRadians,
                state.pendingReset);
        state.pendingReset = null;
        return List.of(outputMeasurement(
                estimated,
                odometry.velocity(),
                timestampSeconds,
                stateStdDevs,
                this));
    }

    private List<Measurement> latest(List<Measurement> measurements) {
        return measurements.stream()
                .max(Comparator.comparingDouble(Measurement::timestampSeconds))
                .map(List::of)
                .orElseGet(List::of);
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
            double variance = finiteVariance(sample.stdDevs().translationVariance(), 1.0);
            double weight = 1.0 / variance;
            x += sample.pose().xMeters() * weight;
            y += sample.pose().yMeters() * weight;
            sin += Math.sin(sample.pose().headingRadians()) * weight;
            cos += Math.cos(sample.pose().headingRadians()) * weight;
            vx += sample.speeds().xMetersPerSecond() * weight;
            vy += sample.speeds().yMetersPerSecond() * weight;
            omega += sample.speeds().angularRadiansPerSecond() * weight;
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
                Math.sqrt(1.0 / totalWeight),
                Math.sqrt(1.0 / totalWeight),
                samples.stream().map(PoseMeasurementSample::stdDevs)
                        .mapToDouble(MeasurementStdDevs::headingRadians).min().orElse(1.0));
        return List.of(outputMeasurement(
                new PoseSnapshot(x / totalWeight, y / totalWeight, Math.atan2(sin, cos)),
                new RobotVelocity(vx / totalWeight, vy / totalWeight, omega / totalWeight),
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
        List<PoseMeasurementSample> samples = poseSamples(measurements);
        if (samples.isEmpty()) {
            return samples;
        }
        PoseMeasurementSample newest = samples.stream()
                .max(Comparator.comparingDouble(PoseMeasurementSample::timestampSeconds))
                .orElseThrow();
        List<PoseMeasurementSample> recent = samples.stream()
                .filter(sample -> newest.timestampSeconds() - sample.timestampSeconds() <= groupWindowSeconds)
                .toList();
        List<PoseMeasurementSample> best = List.of();
        for (PoseMeasurementSample anchor : recent) {
            List<PoseMeasurementSample> cluster = recent.stream()
                    .filter(sample -> agrees(anchor, sample))
                    .toList();
            if (betterCluster(cluster, best)) {
                best = cluster;
            }
        }
        return distinctSourceCount(best) >= minimumSourceCount ? best : List.of();
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
        return sample.stdDevs() == null || sample.stdDevs().headingRadians() <= Math.PI;
    }

    private static boolean trustsTranslation(PoseMeasurementSample sample) {
        return sample.stdDevs() == null
                || (sample.stdDevs().xMeters() < 1.0e6 && sample.stdDevs().yMeters() < 1.0e6);
    }

    private static boolean betterCluster(
            List<PoseMeasurementSample> candidate,
            List<PoseMeasurementSample> current) {
        int candidateSources = distinctSourceCount(candidate);
        int currentSources = distinctSourceCount(current);
        if (candidateSources != currentSources) {
            return candidateSources > currentSources;
        }
        if (candidate.size() != current.size()) {
            return candidate.size() > current.size();
        }
        int candidateTargets = candidate.stream().mapToInt(PoseMeasurementSample::targetCount).sum();
        int currentTargets = current.stream().mapToInt(PoseMeasurementSample::targetCount).sum();
        if (candidateTargets != currentTargets) {
            return candidateTargets > currentTargets;
        }
        double candidateAmbiguity = candidate.stream().mapToDouble(PoseMeasurementSample::ambiguity).sum();
        double currentAmbiguity = current.stream().mapToDouble(PoseMeasurementSample::ambiguity).sum();
        if (Double.compare(candidateAmbiguity, currentAmbiguity) != 0) {
            return candidateAmbiguity < currentAmbiguity;
        }
        return candidate.stream().mapToDouble(PoseMeasurementSample::timestampSeconds).max().orElse(0.0)
                > current.stream().mapToDouble(PoseMeasurementSample::timestampSeconds).max().orElse(0.0);
    }

    private static int distinctSourceCount(List<PoseMeasurementSample> samples) {
        return (int) samples.stream()
                .map(sample -> sample.source() == null ? sample : sample.source())
                .distinct()
                .count();
    }

    private boolean accepts(Measurement measurement, PoseSnapshot pose) {
        for (LocalizationFilter filter : filters) {
            if (!filter.accept(this, measurement, pose)) {
                return false;
            }
        }
        return true;
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
                minimumSourceCount, innovationGate, stateStdDevs, defaultVisionStdDevs, state);
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
                gate, processStdDevs, visionStdDevs, state);
    }

    private static PoseSignal snapshot(List<Measurement> measurements) {
        List<Measurement> values = List.copyOf(measurements);
        return () -> values;
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
                .timing(timestampSeconds, 0.0)
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
                .timing(timestampSeconds, 0.0)
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

    private static final class State {
        private List<Measurement> output = List.of();
        private List<Measurement> accepted = List.of();
        private List<Measurement> rejected = List.of();
        private PoseSnapshot pendingReset;
        private double lastRefreshTimestamp = Double.NaN;
        private boolean runtimeOwned;
        private KalmanEngine kalman;
        private StructPublisher<Pose2d> posePublisher;
    }

    private static final class KalmanEngine {
        private static final int RECOVERY_SAMPLE_COUNT = 3;
        private static final double RECOVERY_MAX_INTERVAL_SECONDS = 0.25;
        private static final double RECOVERY_TRANSLATION_METERS = 0.75;
        private static final double RECOVERY_HEADING_RADIANS = Math.toRadians(25.0);
        private final SwerveDriveKinematics kinematics;
        private final MeasurementStdDevs stateStdDevs;
        private final MeasurementStdDevs defaultVisionStdDevs;
        private final Map<Object, Double> lastSeenVisionTimestamps = new IdentityHashMap<>();
        private final Map<Object, VisionRecovery> visionRecovery = new IdentityHashMap<>();
        private SwerveDrivePoseEstimator estimator;

        private KalmanEngine(
                SwerveOdometry odometry,
                MeasurementStdDevs stateStdDevs,
                MeasurementStdDevs defaultVisionStdDevs) {
            this.stateStdDevs = stateStdDevs;
            this.defaultVisionStdDevs = defaultVisionStdDevs;
            kinematics = new SwerveDriveKinematics(odometry.kinematics().modules().stream()
                    .map(module -> new Translation2d(module.xMeters(), module.yMeters()))
                    .toArray(Translation2d[]::new));
        }

        private PoseSnapshot update(
                SwerveOdometry odometry,
                List<Measurement> vision,
                double timestampSeconds,
                double innovationGate,
                double maxHeadingResidualRadians,
                PoseSnapshot reset) {
            Rotation2d heading = new Rotation2d(odometry.headingRadians());
            SwerveModulePosition[] positions = positions(odometry);
            if (estimator == null) {
                PoseSnapshot initial = reset == null ? odometry.pose() : reset;
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
            vision.stream()
                    .filter(PoseMeasurementSample.class::isInstance)
                    .map(PoseMeasurementSample.class::cast)
                    .sorted(Comparator.comparingDouble(PoseMeasurementSample::timestampSeconds))
                    .forEach(sample -> addVision(sample, innovationGate, maxHeadingResidualRadians));
            return fromWpilib(estimator.getEstimatedPosition());
        }

        private void addVision(
                PoseMeasurementSample sample,
                double innovationGate,
                double maxHeadingResidualRadians) {
            Object source = sample.source() == null ? sample : sample.source();
            double previous = lastSeenVisionTimestamps.getOrDefault(source, Double.NEGATIVE_INFINITY);
            if (sample.timestampSeconds() <= previous) {
                return;
            }
            lastSeenVisionTimestamps.put(source, sample.timestampSeconds());
            MeasurementStdDevs stdDevs = usable(sample.stdDevs()) ? sample.stdDevs() : defaultVisionStdDevs;
            if (headingResidual(sample) > maxHeadingResidualRadians) {
                visionRecovery.remove(source);
                return;
            }
            if (Double.isFinite(innovationGate) && innovation(sample, stdDevs) > innovationGate) {
                VisionRecovery recovery = visionRecovery.computeIfAbsent(source, ignored -> new VisionRecovery());
                if (!recovery.accepts(sample)) {
                    return;
                }
            } else {
                visionRecovery.remove(source);
            }
            estimator.addVisionMeasurement(toWpilib(sample.pose()), sample.timestampSeconds(), vector(stdDevs));
        }

        private static final class VisionRecovery {
            private PoseSnapshot previousPose;
            private double previousTimestamp = Double.NEGATIVE_INFINITY;
            private int consistentSamples;

            private boolean accepts(PoseMeasurementSample sample) {
                boolean strongObservation = sample.targetCount() >= 2;
                boolean continuous = previousPose != null
                        && sample.timestampSeconds() - previousTimestamp <= RECOVERY_MAX_INTERVAL_SECONDS
                        && translationDistance(previousPose, sample.pose()) <= RECOVERY_TRANSLATION_METERS
                        && Math.abs(wrapRadians(previousPose.headingRadians() - sample.pose().headingRadians()))
                                <= RECOVERY_HEADING_RADIANS;
                consistentSamples = strongObservation && continuous ? consistentSamples + 1 : strongObservation ? 1 : 0;
                previousPose = sample.pose();
                previousTimestamp = sample.timestampSeconds();
                return consistentSamples >= RECOVERY_SAMPLE_COUNT;
            }
        }

        private double innovation(PoseMeasurementSample sample, MeasurementStdDevs stdDevs) {
            Pose2d predicted = estimator.sampleAt(sample.timestampSeconds()).orElse(estimator.getEstimatedPosition());
            double dx = sample.pose().xMeters() - predicted.getX();
            double dy = sample.pose().yMeters() - predicted.getY();
            double dh = wrapRadians(sample.pose().headingRadians() - predicted.getRotation().getRadians());
            return square(dx / stdDevs.xMeters())
                    + square(dy / stdDevs.yMeters())
                    + square(dh / stdDevs.headingRadians());
        }

        private double headingResidual(PoseMeasurementSample sample) {
            Pose2d predicted = estimator.sampleAt(sample.timestampSeconds()).orElse(estimator.getEstimatedPosition());
            return Math.abs(wrapRadians(
                    sample.pose().headingRadians() - predicted.getRotation().getRadians()));
        }


        private static SwerveModulePosition[] positions(SwerveOdometry odometry) {
            return odometry.modulePositions().stream()
                    .map(position -> new SwerveModulePosition(
                            position.distanceMeters(),
                            Rotation2d.fromRotations(position.angleRotations())))
                    .toArray(SwerveModulePosition[]::new);
        }

        private static boolean usable(MeasurementStdDevs stdDevs) {
            return stdDevs != null
                    && Double.isFinite(stdDevs.xMeters())
                    && Double.isFinite(stdDevs.yMeters())
                    && Double.isFinite(stdDevs.headingRadians());
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
                    square(stdDevs.xMeters()),
                    square(stdDevs.yMeters()),
                    square(stdDevs.headingRadians()),
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
                    new RobotVelocity(
                            (speeds.xMetersPerSecond() + other.speeds.xMetersPerSecond()) / 2.0,
                            (speeds.yMetersPerSecond() + other.speeds.yMetersPerSecond()) / 2.0,
                            (speeds.angularRadiansPerSecond() + other.speeds.angularRadiansPerSecond()) / 2.0),
                    Math.max(timestampSeconds, other.timestampSeconds),
                    xVar,
                    yVar,
                    hVar,
                    Math.max(ambiguity, other.ambiguity),
                    targetCount + other.targetCount,
                    averageFinite(averageTargetDistanceMeters, other.averageTargetDistanceMeters));
        }

        private MeasurementStdDevs stdDevs() {
            return MeasurementStdDevs.of(
                    Math.sqrt(xVariance),
                    Math.sqrt(yVariance),
                    Math.sqrt(headingVariance));
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
        return VecBuilder.fill(stdDevs.xMeters(), stdDevs.yMeters(), stdDevs.headingRadians());
    }

    private static Pose2d toWpilib(PoseSnapshot pose) {
        return new Pose2d(pose.xMeters(), pose.yMeters(), new Rotation2d(pose.headingRadians()));
    }

    private static PoseSnapshot fromWpilib(Pose2d pose) {
        return new PoseSnapshot(pose.getX(), pose.getY(), pose.getRotation().getRadians());
    }
}
