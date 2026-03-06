package ca.frc6390.athena.sensors.camera.helios;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;

import ca.frc6390.athena.sensors.camera.FiducialSource;
import ca.frc6390.athena.sensors.camera.HeliOSCamera;
import ca.frc6390.athena.sensors.camera.LocalizationSource;
import ca.frc6390.athena.sensors.camera.TargetingSource;
import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.sensors.camera.VisionCameraCapability;
import ca.frc6390.athena.sensors.camera.VisionCameraConfig;
import ca.pd.lib.helios.HeliOS;
import ca.pd.lib.helios.HeliOSLocalization;
import ca.pd.lib.helios.localization.HeliosLocalizationDetectionPose;
import ca.pd.lib.helios.localization.HeliosLocalizationSolveResponse;
import ca.pd.lib.helios.localization.HeliosLocalizationSolverPose;
import ca.pd.lib.helios.localization.HeliosLocalizationSolverResult;
import ca.pd.lib.helios.localization.HeliosLocalizationSourceSampleStatus;
import ca.pd.lib.helios.localization.HeliosLocalizationUtil;
import ca.pd.lib.helios.localization.HeliosLocalizationVector;
import ca.pd.lib.helios.vision.HeliosVisionHelpers;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Athena camera adapter backed by PDLib's HeliOS read-only API.
 */
public class HeliOSCameraAdapter implements HeliOSCamera, LocalizationSource, TargetingSource, FiducialSource {

    private static final Matrix<N3, N1> DEFAULT_STD_DEVS = VecBuilder.fill(0.7, 0.7, Math.toRadians(12));

    private final HeliOSConfig config;
    private final HeliOS helios;
    private final VisionCamera localizationCamera;
    private final AprilTagFieldLayout fieldLayout;
    private boolean connected;
    private LocalizationSnapshot latestSnapshot = LocalizationSnapshot.empty();
    private double lastTargetYawDegrees = Double.NaN;
    private double lastTargetPitchDegrees = Double.NaN;
    private double lastTargetDistanceMeters = Double.NaN;
    private int lastTargetId = -1;
    private double lastPoseAmbiguity = Double.NaN;
    private double lastCameraPitchDegrees = Double.NaN;
    private double lastCameraRollDegrees = Double.NaN;
    private List<VisionCamera.TargetMeasurement> latestMeasurements = List.of();

    private static final class LocalizationSnapshot {
        private final Pose2d pose;
        private final Pose3d pose3d;
        private final double latencySeconds;
        private final int visibleTargets;
        private final double averageDistanceMeters;
        private final Matrix<N3, N1> stdDevs;

        private LocalizationSnapshot(
                Pose2d pose,
                Pose3d pose3d,
                double latencySeconds,
                int visibleTargets,
                double averageDistanceMeters,
                Matrix<N3, N1> stdDevs) {
            this.pose = pose;
            this.pose3d = pose3d;
            this.latencySeconds = latencySeconds;
            this.visibleTargets = visibleTargets;
            this.averageDistanceMeters = averageDistanceMeters;
            this.stdDevs = stdDevs;
        }

        static LocalizationSnapshot empty() {
            return new LocalizationSnapshot(new Pose2d(), new Pose3d(), 0.0, 0, Double.MAX_VALUE, DEFAULT_STD_DEVS);
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

        double averageDistanceMeters() {
            return averageDistanceMeters;
        }

        Matrix<N3, N1> stdDevs() {
            return stdDevs;
        }
    }

    public HeliOSCameraAdapter(HeliOSConfig config) {
        this.helios = new HeliOS(config.target());
        // Keep construction side-effect free. Resolving hostnames over the network here can block
        // RobotCore startup before mechanisms and inputs are registered.
        this.config = config;
        this.fieldLayout = loadFieldLayout(this.config.fieldLayout());
        this.localizationCamera = createVisionCamera();
    }

    public HeliOSConfig getConfig() {
        return config;
    }

    public VisionCamera getVisionCamera() {
        return localizationCamera;
    }

    private VisionCamera createVisionCamera() {
        VisionCameraConfig cameraConfig =
                VisionCameraConfig.create(config.getTable(), config.getSoftware())
                        .localization(l -> l
                                .useForLocalization(config.useForLocalization())
                                .trustDistance(config.trustDistance())
                                .roles(config.roles())
                                .singleStdDevs(DEFAULT_STD_DEVS)
                                .multiStdDevs(DEFAULT_STD_DEVS)
                                .confidenceSupplier(config::confidence))
                        .sources(s -> s
                                .connectedSupplier(this::isConnectedInternal)
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
                                .targetYawSupplier(this::supplyTargetYaw)
                                .targetPitchSupplier(this::supplyTargetPitch)
                                .tagDistanceSupplier(this::supplyTargetDistance)
                                .tagIdSupplier(this::supplyTargetId)
                                .poseAmbiguitySupplier(this::supplyPoseAmbiguity)
                                .cameraPitchSupplier(this::supplyCameraPitch)
                                .cameraRollSupplier(this::supplyCameraRoll)
                                .targetMeasurementsSupplier(this::supplyTargetMeasurements)
                                .fieldLayout(fieldLayout));
        return new VisionCamera(cameraConfig)
                .registerCapability(VisionCameraCapability.HELIOS_CAMERA, this)
                .registerCapability(VisionCameraCapability.LOCALIZATION_SOURCE, this)
                .registerCapability(VisionCameraCapability.TARGETING_SOURCE, this)
                .registerCapability(VisionCameraCapability.FIDUCIAL_SOURCE, this);
    }

    private void refreshLocalizationSnapshot() {
        long t0 = System.nanoTime();
        try {
            HeliOSLocalization localization = resolveLocalization();
            HeliosLocalizationSolveResponse solve = localization.solve();
            long t1 = System.nanoTime();

            HeliosLocalizationSolverResult solver = selectSolver(solve, config.solverId());
            List<HeliosLocalizationDetectionPose> tagInCamera = extractTagInCamera(solver);
            HeliosLocalizationSolverPose robotInField = extractRobotInField(solver);

            Pose2d pose2d = poseFromSolver(robotInField);
            Pose3d pose3d = pose3dFromSolver(robotInField, pose2d);
            double averageDistance = HeliosLocalizationUtil.averageTagDistanceMeters(tagInCamera);
            if (!Double.isFinite(averageDistance)) {
                averageDistance = Double.MAX_VALUE;
            }
            Matrix<N3, N1> stdDevs =
                    HeliosLocalizationUtil.estimateFieldPoseStdDevs(tagInCamera.size(), averageDistance);
            double httpWallMs = Math.max(0.0, (t1 - t0) / 1e6);
            double maxPollMs = maxSourcePollMs(solve);
            double latencySeconds = Math.max(0.0, (httpWallMs + maxPollMs) / 1000.0);

            latestSnapshot =
                    new LocalizationSnapshot(
                            pose2d,
                            pose3d,
                            latencySeconds,
                            tagInCamera.size(),
                            averageDistance,
                            stdDevs);
            localizationCamera.config().stdDevs(stdDevs, stdDevs);
            updateTargetTelemetry(tagInCamera);
            connected = true;
        } catch (Exception ignored) {
            connected = false;
            latestSnapshot = LocalizationSnapshot.empty();
            localizationCamera.config().stdDevs(DEFAULT_STD_DEVS, DEFAULT_STD_DEVS);
            clearTargetTelemetry();
        }
    }

    private HeliOSLocalization resolveLocalization() throws Exception {
        String profile = sanitize(config.localizationProfile());
        if (profile == null) {
            return helios.localization();
        }
        try {
            return helios.localization(profile);
        } catch (IllegalArgumentException ignored) {
            return helios.localization();
        }
    }

    private HeliosLocalizationSolverResult selectSolver(HeliosLocalizationSolveResponse solve, String solverId) {
        if (solve == null || solve.solvers() == null || solve.solvers().isEmpty()) {
            return null;
        }
        String wanted = sanitize(solverId);
        if (wanted != null) {
            for (HeliosLocalizationSolverResult solver : solve.solvers()) {
                if (solver == null) {
                    continue;
                }
                String id = sanitize(solver.id());
                String name = sanitize(solver.name());
                if (wanted.equals(id) || (name != null && wanted.equalsIgnoreCase(name))) {
                    return solver;
                }
            }
        }
        for (HeliosLocalizationSolverResult solver : solve.solvers()) {
            if (solver != null && solver.outputs() != null) {
                return solver;
            }
        }
        return solve.solvers().get(0);
    }

    private List<HeliosLocalizationDetectionPose> extractTagInCamera(HeliosLocalizationSolverResult solver) {
        if (solver == null || solver.outputs() == null || solver.outputs().tagInCamera() == null) {
            return List.of();
        }
        return solver.outputs().tagInCamera();
    }

    private HeliosLocalizationSolverPose extractRobotInField(HeliosLocalizationSolverResult solver) {
        if (solver == null || solver.outputs() == null) {
            return null;
        }
        return solver.outputs().robotInField();
    }

    private Pose2d poseFromSolver(HeliosLocalizationSolverPose solverPose) {
        if (solverPose == null || solverPose.pose() == null || solverPose.pose().translation() == null) {
            return new Pose2d();
        }
        return HeliosLocalizationUtil.toWpilibPose2dField(solverPose.pose());
    }

    private Pose3d pose3dFromSolver(HeliosLocalizationSolverPose solverPose, Pose2d fallback) {
        if (solverPose == null || solverPose.pose() == null || solverPose.pose().translation() == null) {
            return new Pose3d(fallback);
        }
        HeliosLocalizationVector t = solverPose.pose().translation();
        if (!Double.isFinite(t.x()) || !Double.isFinite(t.y()) || !Double.isFinite(t.z())) {
            return new Pose3d(fallback);
        }
        return new Pose3d(
                t.z(),
                t.x(),
                t.y(),
                new Rotation3d(0.0, 0.0, fallback.getRotation().getRadians()));
    }

    private double maxSourcePollMs(HeliosLocalizationSolveResponse solve) {
        if (solve == null || solve.sources() == null || solve.sources().isEmpty()) {
            return 0.0;
        }
        double maxPollMs = 0.0;
        for (HeliosLocalizationSourceSampleStatus source : solve.sources()) {
            if (source == null || !Double.isFinite(source.pollMs())) {
                continue;
            }
            maxPollMs = Math.max(maxPollMs, source.pollMs());
        }
        return maxPollMs;
    }

    private void updateTargetTelemetry(List<HeliosLocalizationDetectionPose> tagInCamera) {
        latestMeasurements = buildTargetMeasurements(tagInCamera);
        if (latestMeasurements.isEmpty()) {
            clearTargetTelemetry();
            return;
        }
        HeliosLocalizationDetectionPose closest = HeliosVisionHelpers.closestTag(tagInCamera);
        if (closest == null || closest.pose() == null || closest.pose().translation() == null) {
            clearTargetTelemetry();
            return;
        }
        HeliosLocalizationVector translation = closest.pose().translation();
        lastTargetYawDegrees = HeliosVisionHelpers.yawDegFromCameraTranslation(translation);
        lastTargetPitchDegrees = HeliosVisionHelpers.pitchDegFromCameraTranslation(translation);
        lastTargetDistanceMeters = HeliosVisionHelpers.distanceMetersFromCameraTranslation(translation);
        lastTargetId = closest.tagId();
    }

    private List<VisionCamera.TargetMeasurement> buildTargetMeasurements(
            List<HeliosLocalizationDetectionPose> tagInCamera) {
        if (tagInCamera == null || tagInCamera.isEmpty()) {
            return List.of();
        }
        List<VisionCamera.TargetMeasurement> measurements = new ArrayList<>();
        for (HeliosLocalizationDetectionPose detection : tagInCamera) {
            VisionCamera.TargetMeasurement measurement = createMeasurement(detection);
            if (measurement != null) {
                measurements.add(measurement);
            }
        }
        return measurements.isEmpty() ? List.of() : List.copyOf(measurements);
    }

    private VisionCamera.TargetMeasurement createMeasurement(HeliosLocalizationDetectionPose detection) {
        if (detection == null || detection.pose() == null || detection.pose().translation() == null) {
            return null;
        }
        HeliosLocalizationVector t = detection.pose().translation();
        if (!Double.isFinite(t.x()) || !Double.isFinite(t.y()) || !Double.isFinite(t.z())) {
            return null;
        }
        Translation2d translation2d = new Translation2d(t.z(), t.x());
        double yaw = HeliosVisionHelpers.yawDegFromCameraTranslation(t);
        double distance = HeliosVisionHelpers.distanceMetersFromCameraTranslation(t);
        Double yawDegrees = Double.isFinite(yaw) ? yaw : null;
        Double distanceMeters = Double.isFinite(distance) ? distance : null;
        Double confidence = Double.isFinite(config.confidence())
                ? Math.max(0.0, Math.min(1.0, config.confidence()))
                : null;
        return new VisionCamera.TargetMeasurement(
                translation2d,
                yawDegrees,
                distanceMeters,
                detection.tagId(),
                confidence);
    }

    private void clearTargetTelemetry() {
        lastTargetYawDegrees = Double.NaN;
        lastTargetPitchDegrees = Double.NaN;
        lastTargetDistanceMeters = Double.NaN;
        lastTargetId = -1;
        lastPoseAmbiguity = Double.NaN;
        lastCameraPitchDegrees = Double.NaN;
        lastCameraRollDegrees = Double.NaN;
        latestMeasurements = List.of();
    }

    private boolean isConnectedInternal() {
        return connected;
    }

    private boolean computeHasTargets() {
        return connected && !latestMeasurements.isEmpty();
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

    private int supplyVisibleTargets() {
        return latestSnapshot.visibleTargets();
    }

    private double supplyAverageDistance() {
        return latestSnapshot.averageDistanceMeters();
    }

    private double supplyTargetYaw() {
        return lastTargetYawDegrees;
    }

    private double supplyTargetPitch() {
        return lastTargetPitchDegrees;
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

    private void setRobotOrientation(Pose2d pose) {
        // HeliOS solve endpoint is read-only and does not currently accept orientation hints.
    }

    private static String sanitize(String value) {
        if (value == null) {
            return null;
        }
        String trimmed = value.trim();
        return trimmed.isEmpty() ? null : trimmed;
    }

    private static AprilTagFieldLayout loadFieldLayout(edu.wpi.first.apriltag.AprilTagFields layout) {
        try {
            return AprilTagFieldLayout.loadField(layout);
        } catch (Exception ignored) {
            return null;
        }
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
    public Matrix<N3, N1> getLocalizationStdDevs() {
        return localizationCamera.getLocalizationStdDevs();
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
}
