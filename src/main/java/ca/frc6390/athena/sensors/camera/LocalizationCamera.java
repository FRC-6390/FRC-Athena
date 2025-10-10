package ca.frc6390.athena.sensors.camera;

import static ca.frc6390.athena.util.SupplierUtil.optionalDouble;
import static ca.frc6390.athena.util.SupplierUtil.optionalInt;
import static ca.frc6390.athena.util.SupplierUtil.wrapBoolean;
import static ca.frc6390.athena.util.SupplierUtil.wrapConsumer;
import static ca.frc6390.athena.util.SupplierUtil.wrapDouble;
import static ca.frc6390.athena.util.SupplierUtil.wrapInt;
import static ca.frc6390.athena.util.SupplierUtil.wrapSupplier;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.EnumSet;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

/**
 * Unified wrapper around vendor-specific vision cameras. Works similarly to {@code MotorController}
 * by delegating to suppliers/consumers defined in {@link LocalizationCameraConfig}.
 */
public class LocalizationCamera {

    public enum CoordinateSpace {
        CAMERA,
        ROBOT,
        FIELD,
        TAG
    }

    public static record TargetMeasurement(
            Translation2d cameraTranslation,
            Double yawDegrees,
            Double distanceMeters,
            int tagId,
            Double confidence) {}

    private final LocalizationCameraConfig config;
    private final BooleanSupplier connectedSupplier;
    private final BooleanSupplier hasTargetsSupplier;
    private final Supplier<Pose2d> poseSupplier;
    private final DoubleSupplier latencySupplier;
    private final IntSupplier visibleTargetsSupplier;
    private final DoubleSupplier averageDistanceSupplier;
    private final Consumer<Pose2d> orientationConsumer;
    private final Supplier<Pose2d> orientationSupplier;
    private final Runnable updateHook;
    private final DoubleSupplier targetYawSupplier;
    private final DoubleSupplier targetPitchSupplier;
    private final DoubleSupplier targetDistanceSupplier;
    private final IntSupplier tagIdSupplier;
    private final DoubleSupplier confidenceSupplier;
    private final Supplier<List<TargetMeasurement>> targetMeasurementsSupplier;
    private final Transform3d robotToCameraTransform;
    private final Transform3d cameraToRobotTransform;
    private final Translation2d robotToCameraTranslation;
    private final Rotation2d cameraYawRelativeToRobot;
    private final double displayHorizontalFovDeg;
    private final double displayRangeMeters;
    private final EnumSet<LocalizationCameraConfig.CameraRole> roles;
    private final EnumMap<LocalizationCameraCapability, Object> capabilities =
            new EnumMap<>(LocalizationCameraCapability.class);
    private final StructPublisher<Pose2d> networkEstimatedPosePublisher;

    private double cachedDistanceMeters = Double.NaN;
    private double cachedYawDegrees = Double.NaN;
    private Optional<Translation2d> cachedCameraTranslation = Optional.empty();
    private List<TargetMeasurement> measurementCache = List.of();

    private Matrix<N3, N1> singleStdDevs;
    private Matrix<N3, N1> multiStdDevs;

    public LocalizationCamera(LocalizationCameraConfig config) {
        this.config = config;
        this.connectedSupplier = wrapBoolean(config.getConnectedSupplier(), true);
        this.hasTargetsSupplier = wrapBoolean(config.getHasTargetsSupplier(), false);
        this.poseSupplier = wrapSupplier(config.getPoseSupplier(), Pose2d::new);
        this.latencySupplier = wrapDouble(config.getLatencySupplier(), 0.0);
        this.visibleTargetsSupplier = wrapInt(config.getVisibleTargetsSupplier(), 0);
        this.averageDistanceSupplier = wrapDouble(config.getAverageDistanceSupplier(), Double.MAX_VALUE);
        this.orientationConsumer = wrapConsumer(config.getOrientationConsumer(), pose -> {});
        this.orientationSupplier = config.getOrientationSupplier();
        this.updateHook = config.getUpdateHook();
        this.targetYawSupplier = wrapDouble(config.getTargetYawSupplier(), Double.NaN);
        this.targetPitchSupplier = wrapDouble(config.getTargetPitchSupplier(), Double.NaN);
        this.targetDistanceSupplier = wrapDouble(config.getTagDistanceSupplier(), Double.NaN);
        this.tagIdSupplier = wrapInt(config.getTagIdSupplier(), -1);
        this.confidenceSupplier = wrapDouble(config.getConfidenceSupplier(), 1.0);
        this.targetMeasurementsSupplier = config.getTargetMeasurementsSupplier();
        this.robotToCameraTransform = config.getRobotToCameraTransform();
        this.cameraToRobotTransform = config.getCameraToRobotTransform();
        this.robotToCameraTranslation =
                new Translation2d(robotToCameraTransform.getX(), robotToCameraTransform.getY());
        this.cameraYawRelativeToRobot =
                Rotation2d.fromRadians(robotToCameraTransform.getRotation().getZ());
        this.displayHorizontalFovDeg = config.getDisplayHorizontalFov();
        this.displayRangeMeters = config.getDisplayRangeMeters();
        EnumSet<LocalizationCameraConfig.CameraRole> configRoles = config.getRoles();
        this.roles = configRoles.isEmpty()
                ? EnumSet.noneOf(LocalizationCameraConfig.CameraRole.class)
                : EnumSet.copyOf(configRoles);
        if (config.isUseForLocalization()) {
            this.roles.add(LocalizationCameraConfig.CameraRole.LOCALIZATION);
        }
        this.singleStdDevs = config.getSingleStdDevs();
        this.multiStdDevs = config.getMultiTagStdDevs();
        capabilities.putAll(config.capabilities());
        String ntBase = "/Vision/Cameras/" + config.getTable();
        networkEstimatedPosePublisher = NetworkTableInstance.getDefault()
                .getStructTopic(ntBase + "/EstimatedPose", Pose2d.struct)
                .publish();
        networkEstimatedPosePublisher.set(new Pose2d());
    }

    public LocalizationCameraConfig getConfig() {
        return config;
    }

    public LocalizationCamera update() {
        if (updateHook != null) {
            updateHook.run();
        }
        if (orientationSupplier != null) {
            Pose2d pose = orientationSupplier.get();
            if (pose != null) {
                orientationConsumer.accept(pose);
            }
        }
        List<TargetMeasurement> measurements =
                targetMeasurementsSupplier != null ? targetMeasurementsSupplier.get() : List.of();
        measurementCache = measurements != null ? List.copyOf(measurements) : List.of();
        Pose2d estimatedPose = poseSupplier.get();
        networkEstimatedPosePublisher.set(estimatedPose != null ? estimatedPose : new Pose2d());
        return this;
    }

    public boolean isConnected() {
        return connectedSupplier.getAsBoolean();
    }

    public boolean hasValidTarget() {
        return isConnected() && hasTargetsSupplier.getAsBoolean();
    }

    public Pose2d getLocalizationPose() {
        update();
        Pose2d pose = poseSupplier.get();
        return pose != null ? pose : new Pose2d();
    }

    public double getLocalizationLatency() {
        update();
        return latencySupplier.getAsDouble();
    }

    public void setRobotOrientation(Pose2d pose) {
        if (pose != null) {
            orientationConsumer.accept(pose);
        }
    }

    public Matrix<N3, N1> getLocalizationStdDevs() {
        update();
        int targets = visibleTargetsSupplier.getAsInt();
        double avgDistance = averageDistanceSupplier.getAsDouble();
        return recalculateStdDevs(targets, avgDistance, config.getTrustDistance());
    }

    public void setStdDevs(Matrix<N3, N1> single, Matrix<N3, N1> multi) {
        if (single != null) {
            singleStdDevs = single;
        }
        if (multi != null) {
            multiStdDevs = multi;
        }
    }

    public Matrix<N3, N1> getSingleStdDev() {
        return singleStdDevs;
    }

    public Matrix<N3, N1> getMultiStdDev() {
        return multiStdDevs;
    }

    public Matrix<N3, N1> recalculateStdDevs(int numTags, double avgDist, double trustDistance) {
        if (numTags <= 0) {
            return singleStdDevs;
        }
        double distance = avgDist;
        if (numTags > 0) {
            distance = avgDist / numTags;
        }
        if (numTags > 1) {
            return multiStdDevs;
        }
        if (distance > trustDistance) {
            return VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }

        double scale = 1 + (distance * distance / 30.0);
        return singleStdDevs.times(scale);
    }

    public LocalizationData getLocalizationData() {
        update();
        Pose2d pose = poseSupplier.get();
        double latency = latencySupplier.getAsDouble();
        Matrix<N3, N1> stdDevs =
                recalculateStdDevs(
                        visibleTargetsSupplier.getAsInt(),
                        averageDistanceSupplier.getAsDouble(),
                        config.getTrustDistance());
        return new LocalizationData(pose != null ? pose : new Pose2d(), latency, stdDevs);
    }

    public boolean isUseForLocalization() {
        return config.isUseForLocalization();
    }

    public void setUseForLocalization(boolean useForLocalization) {
        config.setUseForLocalization(useForLocalization);
        if (useForLocalization) {
            roles.add(LocalizationCameraConfig.CameraRole.LOCALIZATION);
        } else {
            roles.remove(LocalizationCameraConfig.CameraRole.LOCALIZATION);
        }
    }

    public LocalizationCamera registerCapability(LocalizationCameraCapability capabilityKey, Object implementation) {
        if (capabilityKey != null && implementation != null) {
            if (!capabilityKey.getType().isInstance(implementation)) {
                throw new IllegalArgumentException(
                        "Capability " + capabilityKey + " expects implementation of type "
                                + capabilityKey.getType().getName()
                                + " but received "
                                + implementation.getClass().getName());
            }
            capabilities.put(capabilityKey, implementation);
        }
        return this;
    }

    public <T> Optional<T> capability(LocalizationCameraCapability capabilityKey) {
        return capabilityKey.cast(capabilities.get(capabilityKey));
    }

    public boolean supports(LocalizationCameraCapability capabilityKey) {
        return capabilities.containsKey(capabilityKey);
    }

    public EnumSet<LocalizationCameraConfig.CameraRole> getRoles() {
        return EnumSet.copyOf(roles);
    }

    public double getMeasurementConfidence() {
        return confidenceSupplier.getAsDouble();
    }

    public Transform3d getCameraToRobotTransform() {
        return cameraToRobotTransform;
    }

    public Transform3d getRobotToCameraTransform() {
        return robotToCameraTransform;
    }

    public double getDisplayHorizontalFovDeg() {
        return displayHorizontalFovDeg;
    }

    public double getDisplayRangeMeters() {
        return displayRangeMeters;
    }

    /**
     * Latest horizontal angle from the camera to the target in degrees. Empty when no target is
     * available or the vendor does not provide yaw data.
     */
    public OptionalDouble getTargetYawDegrees() {
        update();
        return optionalDouble(targetYawSupplier);
    }

    /**
     * Latest vertical angle from the camera to the target in degrees. Empty when no target is
     * available or the vendor does not provide pitch data.
     */
    public OptionalDouble getTargetPitchDegrees() {
        update();
        return optionalDouble(targetPitchSupplier);
    }

    /**
     * Latest distance estimate from the camera to the target in meters. Empty when no estimate is
     * available.
     */
    public OptionalDouble getTargetDistanceMeters() {
        update();
        return optionalDouble(targetDistanceSupplier);
    }

    /**
     * Latest observed tag/fiducial ID. Empty when no target is currently visible.
     */
    public OptionalInt getLatestTagId() {
        update();
        return optionalInt(tagIdSupplier);
    }

    /** Returns the camera-to-target translation in camera coordinates. */
    public Optional<Translation2d> getCameraRelativeTranslation() {
        update();
        return getPrimaryCameraTranslation();
    }

    /** Returns the latest observation using camera space. */
    public Optional<TargetObservation> getLatestObservation() {
        return getLatestObservation(CoordinateSpace.CAMERA, null, null);
    }

    /**
     * Returns the latest observation in the requested coordinate space.
     *
     * @param space target coordinate system
     * @param robotPose robot pose in field space when needed (FIELD/TAG conversions)
     * @param tagPose tag pose in field space when space is TAG
     */
    public Optional<TargetObservation> getLatestObservation(CoordinateSpace space, Pose2d robotPose, Pose2d tagPose) {
        List<TargetObservation> observations = getObservations(space, robotPose, tagPose);
        return observations.isEmpty() ? Optional.empty() : Optional.of(observations.get(0));
    }

    /**
     * Returns all current target observations in the requested coordinate space. The first element is
     * the primary target, matching the behaviour of {@link #getLatestObservation(CoordinateSpace, Pose2d, Pose2d)}.
     */
    public List<TargetObservation> getObservations(CoordinateSpace space, Pose2d robotPose, Pose2d tagPose) {
        update();
        List<TargetMeasurement> measurements = measurementCache;
        if (measurements.isEmpty()) {
            TargetMeasurement fallback = buildFallbackMeasurement();
            if (fallback == null) {
                return List.of();
            }
            measurements = List.of(fallback);
        }

        Matrix<N3, N1> stdDevs =
                recalculateStdDevs(
                        visibleTargetsSupplier.getAsInt(),
                        averageDistanceSupplier.getAsDouble(),
                        config.getTrustDistance());
        LocalizationData data =
                new LocalizationData(poseSupplier.get(), latencySupplier.getAsDouble(), stdDevs);
        double defaultConfidence = getMeasurementConfidence();

        List<TargetObservation> observations = new ArrayList<>(measurements.size());
        for (TargetMeasurement measurement : measurements) {
            Optional<Translation2d> cameraTranslationOpt = cameraTranslationFromMeasurement(measurement);
            Optional<Translation2d> translatedOpt =
                    cameraTranslationOpt.flatMap(
                            ct -> convertCameraTranslationToSpace(ct, space, robotPose, tagPose));
            Translation2d translated = translatedOpt.orElse(null);

            double yaw =
                    measurement.yawDegrees() != null ? measurement.yawDegrees() : Double.NaN;
            if (Double.isNaN(yaw) && cameraTranslationOpt.isPresent()) {
                Translation2d ct = cameraTranslationOpt.get();
                yaw = Math.toDegrees(-Math.atan2(ct.getY(), ct.getX()));
            }

            double distance =
                    measurement.distanceMeters() != null ? measurement.distanceMeters() : Double.NaN;
            if (Double.isNaN(distance) && cameraTranslationOpt.isPresent()) {
                distance = cameraTranslationOpt.get().getNorm();
            }

            double confidence =
                    measurement.confidence() != null && !Double.isNaN(measurement.confidence())
                            ? measurement.confidence()
                            : defaultConfidence;

            observations.add(
                    new TargetObservation(
                            measurement.tagId(),
                            yaw,
                            distance,
                            translated,
                            space,
                            data,
                            confidence));
        }

        return observations;
    }

    /** Projects the current target translation into field space. */
    public Optional<Translation2d> projectToFieldTranslation(Pose2d robotPose, Translation2d cameraOffsetMeters) {
        if (robotPose == null) {
            return Optional.empty();
        }
        update();
        return getPrimaryCameraTranslation().map(cameraToTarget -> {
            Translation2d offset = cameraOffsetMeters != null ? cameraOffsetMeters : robotToCameraTranslation;
            Translation2d robotRelative = toRobotTranslation(cameraToTarget, offset);
            return robotPose.getTranslation().plus(robotRelative.rotateBy(robotPose.getRotation()));
        });
    }

    /** Estimates the robot pose using the provided tag pose and current measurement. */
    public Optional<Pose2d> estimateFieldPoseFromTag(Pose2d tagPose, Translation2d cameraOffsetMeters) {
        if (tagPose == null) {
            return Optional.empty();
        }
        update();
        return getPrimaryCameraTranslation().map(cameraToTarget -> {
            var robotRotation = getLocalizationPose().getRotation();
            Translation2d offset = cameraOffsetMeters != null ? cameraOffsetMeters : robotToCameraTranslation;
            Translation2d robotToTagRobot = toRobotTranslation(cameraToTarget, offset);
            Translation2d robotToTagField = robotToTagRobot.rotateBy(robotRotation);
            Translation2d robotTranslation = tagPose.getTranslation().minus(robotToTagField);
            return new Pose2d(robotTranslation, robotRotation);
        });
    }

    /**
     * Returns the target translation in the requested coordinate space.
     */
    public Optional<Translation2d> getTargetTranslation(CoordinateSpace space, Pose2d robotPose, Pose2d tagPose) {
        List<TargetObservation> observations = getObservations(space, robotPose, tagPose);
        if (observations.isEmpty()) {
            return Optional.empty();
        }
        return Optional.ofNullable(observations.get(0).translation());
    }

    private Optional<Translation2d> getPrimaryCameraTranslation() {
        for (TargetMeasurement measurement : measurementCache) {
            Optional<Translation2d> translation = cameraTranslationFromMeasurement(measurement);
            if (translation.isPresent()) {
                return translation;
            }
        }
        return computeFallbackCameraTranslation();
    }

    private TargetMeasurement buildFallbackMeasurement() {
        Optional<Translation2d> translationOpt = computeFallbackCameraTranslation();
        OptionalDouble yawOpt = optionalDouble(targetYawSupplier);
        OptionalDouble distanceOpt = optionalDouble(targetDistanceSupplier);
        OptionalInt tagIdOpt = optionalInt(tagIdSupplier);
        boolean hasData =
                translationOpt.isPresent()
                        || yawOpt.isPresent()
                        || distanceOpt.isPresent()
                        || tagIdOpt.isPresent();
        if (!hasData) {
            return null;
        }
        return new TargetMeasurement(
                translationOpt.orElse(null),
                yawOpt.isPresent() ? yawOpt.getAsDouble() : Double.NaN,
                distanceOpt.isPresent() ? distanceOpt.getAsDouble() : Double.NaN,
                tagIdOpt.orElse(-1),
                Double.NaN);
    }

    /**
     * Camera space translation (X forward, Y left) based on vendor yaw/distance. Vendor APIs such as
     * Limelight and PhotonVision report positive yaw when the target sits to the right of the camera,
     * so we invert the supplied yaw to align with the WPILib convention where positive Y is left.
     */
    private Optional<Translation2d> computeFallbackCameraTranslation() {
        double distanceValue = targetDistanceSupplier.getAsDouble();
        double yawDegrees = targetYawSupplier.getAsDouble();
        if (Double.isNaN(distanceValue) || Double.isNaN(yawDegrees)) {
            cachedDistanceMeters = Double.NaN;
            cachedYawDegrees = Double.NaN;
            cachedCameraTranslation = Optional.empty();
            return Optional.empty();
        }
        if (!Double.isNaN(cachedDistanceMeters)
                && !Double.isNaN(cachedYawDegrees)
                && cachedCameraTranslation.isPresent()
                && distanceValue == cachedDistanceMeters
                && yawDegrees == cachedYawDegrees) {
            return cachedCameraTranslation;
        }

        double yawRadians = Math.toRadians(-yawDegrees);
        double x = distanceValue * Math.cos(yawRadians);
        double y = distanceValue * Math.sin(yawRadians);
        Translation2d translation = new Translation2d(x, y);
        cachedDistanceMeters = distanceValue;
        cachedYawDegrees = yawDegrees;
        cachedCameraTranslation = Optional.of(translation);
        return cachedCameraTranslation;
    }

    private Optional<Translation2d> cameraTranslationFromMeasurement(TargetMeasurement measurement) {
        if (measurement == null) {
            return Optional.empty();
        }
        if (measurement.cameraTranslation() != null) {
            return Optional.of(measurement.cameraTranslation());
        }
        Double yaw = measurement.yawDegrees();
        Double distance = measurement.distanceMeters();
        if (yaw == null || distance == null || Double.isNaN(yaw) || Double.isNaN(distance)) {
            return Optional.empty();
        }
        double yawRadians = Math.toRadians(-yaw);
        double x = distance * Math.cos(yawRadians);
        double y = distance * Math.sin(yawRadians);
        return Optional.of(new Translation2d(x, y));
    }

    private Optional<Translation2d> convertCameraTranslationToSpace(
            Translation2d cameraTranslation, CoordinateSpace space, Pose2d robotPose, Pose2d tagPose) {
        return switch (space) {
            case CAMERA -> Optional.of(cameraTranslation);
            case ROBOT -> Optional.of(toRobotTranslation(cameraTranslation));
            case FIELD -> {
                if (robotPose == null) {
                    yield Optional.empty();
                }
                Translation2d robotRelative = toRobotTranslation(cameraTranslation);
                yield Optional.of(
                        robotPose.getTranslation()
                                .plus(robotRelative.rotateBy(robotPose.getRotation())));
            }
            case TAG -> {
                if (tagPose == null) {
                    yield Optional.empty();
                }
                Optional<Pose2d> robotPoseEstimate =
                        estimateFieldPoseFromTag(tagPose, robotToCameraTranslation);
                if (robotPoseEstimate.isEmpty()) {
                    yield Optional.empty();
                }
                Pose2d robotInTag = robotPoseEstimate.get().relativeTo(tagPose);
                yield Optional.of(robotInTag.getTranslation());
            }
        };
    }

    private Translation2d toRobotTranslation(Translation2d cameraTranslation) {
        return toRobotTranslation(cameraTranslation, robotToCameraTranslation);
    }

    private Translation2d toRobotTranslation(Translation2d cameraTranslation, Translation2d offsetOverride) {
        Translation2d rotated = cameraTranslation.rotateBy(cameraYawRelativeToRobot);
        Translation2d cameraOffset = offsetOverride != null ? offsetOverride : robotToCameraTranslation;
        return rotated.plus(cameraOffset);
    }

    public static record LocalizationData(Pose2d pose, double latency, Matrix<N3, N1> stdDevs) {}

    public static record TargetObservation(int tagId, double yawDegrees, double distanceMeters,
                                           Translation2d translation, CoordinateSpace space,
                                           LocalizationData localizationData, double confidence) {
        public boolean hasDistance() {
            return !Double.isNaN(distanceMeters);
        }

        public boolean hasYaw() {
            return !Double.isNaN(yawDegrees);
        }

        public boolean hasTranslation() {
            return translation != null;
        }

        public boolean isSpace(CoordinateSpace expectedSpace) {
            return space == expectedSpace;
        }

        public boolean hasConfidence() {
            return !Double.isNaN(confidence);
        }
    }

    public interface LedControl {
        void setLedMode(LedMode mode);
        LedMode getLedMode();
    }

    public interface PipelineControl {
        void setPipeline(int pipeline);
        int getPipeline();
    }

    public interface StreamControl {
        void setStreamMode(StreamMode mode);
        StreamMode getStreamMode();
    }

    public enum LedMode {
        PIPELINE,
        OFF,
        BLINK,
        ON
    }

    public enum StreamMode {
        STANDARD,
        PIP_MAIN,
        PIP_SECONDARY
    }
}
