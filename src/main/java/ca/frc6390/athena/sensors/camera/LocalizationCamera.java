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
import edu.wpi.first.math.geometry.Pose3d;
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

    /**
     * Coordinate systems used when translating measurements emitted by the camera.
     */
    public enum CoordinateSpace {
        /** Measurements expressed relative to the camera frame (X forward, Y left). */
        CAMERA(false, false),
        /** Measurements expressed relative to the robot frame after applying the camera transform. */
        ROBOT(false, false),
        /**
         * Measurements anchored in the global field frame. If no robot pose override is supplied the
         * camera's own estimated pose will be used when available.
         */
        FIELD(true, false),
        /** Measurements expressed relative to the observed tag pose. */
        TAG(false, true);

        private final boolean acceptsRobotPose;
        private final boolean requiresTagPose;

        CoordinateSpace(boolean acceptsRobotPose, boolean requiresTagPose) {
            this.acceptsRobotPose = acceptsRobotPose;
            this.requiresTagPose = requiresTagPose;
        }

        /**
         * True when callers may supply a field-relative robot pose to influence the conversion. When
         * {@code false} any supplied pose is ignored.
         */
        public boolean acceptsRobotPose() {
            return acceptsRobotPose;
        }

        /**
         * True when callers must supply a tag pose expressed in field coordinates for conversions.
         */
        public boolean requiresTagPose() {
            return requiresTagPose;
        }
    }

    /**
     * Snapshot of a single target reading produced by the camera vendor implementation.
     *
     * @param cameraTranslation target translation in camera space when provided
     * @param yawDegrees yaw offset from the camera optical axis in degrees
     * @param distanceMeters direct distance from the camera to the target in meters
     * @param tagId fiducial identifier associated with the target
     * @param confidence vendor supplied confidence score in the range {@code [0, 1]}
     */
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
    private final Supplier<Pose3d> pose3dSupplier;
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

    /**
     * Creates a new camera wrapper using the supplied vendor configuration.
     *
     * @param config provider-specific configuration that exposes the camera capabilities
     */
    public LocalizationCamera(LocalizationCameraConfig config) {
        this.config = config;
        this.connectedSupplier = wrapBoolean(config.getConnectedSupplier(), true);
        this.hasTargetsSupplier = wrapBoolean(config.getHasTargetsSupplier(), false);
        this.poseSupplier = wrapSupplier(config.getPoseSupplier(), Pose2d::new);
        this.pose3dSupplier = wrapSupplier(config.getPose3dSupplier(), Pose3d::new);
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

    /**
     * Returns the underlying configuration that defines how the camera is integrated.
     */
    public LocalizationCameraConfig getConfig() {
        return config;
    }

    /**
     * Runs any periodic hooks, caches vendor measurements, and pushes the estimated pose to
     * NetworkTables. Returns {@code this} to allow fluent access patterns.
     */
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

    /**
     * Indicates whether the camera reports itself as connected.
     *
     * @return {@code true} when the vendor feed is reachable
     */
    public boolean isConnected() {
        return connectedSupplier.getAsBoolean();
    }

    /**
     * True when the camera is connected and currently reporting a valid target.
     */
    public boolean hasValidTarget() {
        return isConnected() && hasTargetsSupplier.getAsBoolean();
    }

    /**
     * Retrieves the pose supplied by the vendor localization pipeline. Triggers a refresh before
     * reading the measurement.
     *
     * @return pose in field coordinates, or an empty pose when the vendor returns {@code null}
     */
    public Pose2d getLocalizationPose() {
        update();
        Pose2d pose = poseSupplier.get();
        return pose != null ? pose : new Pose2d();
    }

    /**
     * Returns the vendor reported pipeline latency in seconds.
     */
    public double getLocalizationLatency() {
        update();
        return latencySupplier.getAsDouble();
    }

    /**
     * Provides the current robot orientation to the camera processor when supported.
     *
     * @param pose robot pose that should align the vendor localization estimate
     */
    public void setRobotOrientation(Pose2d pose) {
        if (pose != null) {
            orientationConsumer.accept(pose);
        }
    }

    /**
     * Computes the measurement noise matrix based on the current target count and distance.
     */
    public Matrix<N3, N1> getLocalizationStdDevs() {
        update();
        int targets = visibleTargetsSupplier.getAsInt();
        double avgDistance = averageDistanceSupplier.getAsDouble();
        return recalculateStdDevs(targets, avgDistance, config.getTrustDistance());
    }

    /**
     * Overrides the single and multi-tag standard deviation matrices used when estimating pose.
     *
     * @param single single-target standard deviation matrix
     * @param multi multi-target standard deviation matrix
     */
    public void setStdDevs(Matrix<N3, N1> single, Matrix<N3, N1> multi) {
        if (single != null) {
            singleStdDevs = single;
        }
        if (multi != null) {
            multiStdDevs = multi;
        }
    }

    /**
     * Returns the currently configured single-target standard deviation matrix.
     */
    public Matrix<N3, N1> getSingleStdDev() {
        return singleStdDevs;
    }

    /**
     * Returns the currently configured multi-target standard deviation matrix.
     */
    public Matrix<N3, N1> getMultiStdDev() {
        return multiStdDevs;
    }

    /**
     * Recalculates the standard deviations based on the number of tags, average distance, and trust
     * thresholds.
     *
     * @param numTags number of visible tags
     * @param avgDist camera reported average distance in meters
     * @param trustDistance distance threshold for trusting single-tag results
     * @return pose covariance diagonal expressed as a {@link Matrix}
     */
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

    /**
     * Convenience wrapper that bundles the latest vendor pose, latency, and covariance into a single
     * immutable object.
     */
    public LocalizationData getLocalizationData() {
        update();
        Pose2d pose = poseSupplier.get();
        Pose3d pose3d = pose3dSupplier.get();
        double latency = latencySupplier.getAsDouble();
        Matrix<N3, N1> stdDevs =
                recalculateStdDevs(
                        visibleTargetsSupplier.getAsInt(),
                        averageDistanceSupplier.getAsDouble(),
                        config.getTrustDistance());
        Pose2d safePose = pose != null ? pose : new Pose2d();
        Pose3d safePose3d = pose3d != null ? pose3d : new Pose3d(safePose);
        return new LocalizationData(safePose, safePose3d, latency, stdDevs);
    }

    /**
     * Returns whether this camera currently participates in robot localization.
     */
    public boolean isUseForLocalization() {
        return config.isUseForLocalization();
    }

    /**
     * Enables or disables use of the camera for pose estimation.
     *
     * @param useForLocalization {@code true} when the camera should contribute to localization
     */
    public void setUseForLocalization(boolean useForLocalization) {
        config.setUseForLocalization(useForLocalization);
        if (useForLocalization) {
            roles.add(LocalizationCameraConfig.CameraRole.LOCALIZATION);
        } else {
            roles.remove(LocalizationCameraConfig.CameraRole.LOCALIZATION);
        }
    }

    /**
     * Registers a capability implementation that can later be queried through
     * {@link #capability(LocalizationCameraCapability)}.
     *
     * @param capabilityKey capability descriptor
     * @param implementation object implementing the capability contract
     * @return this camera for chaining
     * @throws IllegalArgumentException when the implementation does not match the capability type
     */
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

    /**
     * Retrieves the registered capability implementation when available.
     *
     * @param capabilityKey capability descriptor
     * @return optional containing the implementation when installed
     */
    public <T> Optional<T> capability(LocalizationCameraCapability capabilityKey) {
        return capabilityKey.cast(capabilities.get(capabilityKey));
    }

    /**
     * True when a capability implementation has been registered for the given key.
     */
    public boolean supports(LocalizationCameraCapability capabilityKey) {
        return capabilities.containsKey(capabilityKey);
    }

    /**
     * Returns a defensive copy of the roles assigned to this camera instance.
     */
    public EnumSet<LocalizationCameraConfig.CameraRole> getRoles() {
        return EnumSet.copyOf(roles);
    }

    /**
     * Vendor reported confidence associated with the current frame.
     */
    public double getMeasurementConfidence() {
        return confidenceSupplier.getAsDouble();
    }

    /**
     * Transform from the camera frame into the robot frame.
     */
    public Transform3d getCameraToRobotTransform() {
        return cameraToRobotTransform;
    }

    /**
     * Transform from the robot frame into the camera frame.
     */
    public Transform3d getRobotToCameraTransform() {
        return robotToCameraTransform;
    }

    /**
     * Horizontal field of view used by UI overlays, in degrees.
     */
    public double getDisplayHorizontalFovDeg() {
        return displayHorizontalFovDeg;
    }

    /**
     * Maximum visualization range, in meters, for HUD style displays.
     */
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

    /**
     * Returns the camera-to-target translation expressed in camera coordinates, if the vendor
     * reported enough information to compute it.
     */
    public Optional<Translation2d> getCameraRelativeTranslation() {
        update();
        return getPrimaryCameraTranslation();
    }

    /**
     * Returns the latest observation using {@link CoordinateSpace#CAMERA} coordinates for the
     * translation component.
     */
    public Optional<TargetObservation> getLatestObservation() {
        return getLatestObservation(CoordinateSpace.CAMERA, null, null);
    }

    /**
     * Returns the latest observation in the requested coordinate space, relying on internal pose
     * estimates when explicit poses are not required. For {@link CoordinateSpace#FIELD} conversions the
     * camera's estimated pose is used when available.
     *
     * @param space target coordinate system
     */
    public Optional<TargetObservation> getLatestObservation(CoordinateSpace space) {
        return getLatestObservation(space, null, null);
    }

    /**
     * Returns the latest observation in the requested coordinate space using the supplied robot pose
     * when needed. Use {@link #getLatestObservation(CoordinateSpace, Pose2d, Pose2d)} when a tag pose
     * is also required.
     *
     * @param space target coordinate system
     * @param robotPose robot pose in field space when required (FIELD conversions)
     */
    public Optional<TargetObservation> getLatestObservation(CoordinateSpace space, Pose2d robotPose) {
        return getLatestObservation(space, robotPose, null);
    }

    /**
     * Returns the latest observation in the requested coordinate space.
     *
     * @param space target coordinate system
     * @param robotPose robot pose in field space when needed (FIELD/TAG conversions)
     * @param tagPose tag pose in field space when space is TAG
     * Converts the primary measurement into the requested coordinate system and returns it together
     * with any localization metadata.
     */
    public Optional<TargetObservation> getLatestObservation(CoordinateSpace space, Pose2d robotPose, Pose2d tagPose) {
        List<TargetObservation> observations = getObservations(space, robotPose, tagPose);
        return observations.isEmpty() ? Optional.empty() : Optional.of(observations.get(0));
    }

    /**
     * Returns all current target observations in the requested coordinate space. The first element is
     * the primary target, matching the behaviour of {@link #getLatestObservation(CoordinateSpace, Pose2d, Pose2d)}.
     *
     * @param space coordinate space to convert into
     * @param robotPose robot pose in field space when required (FIELD/TAG conversions)
     * @param tagPose tag pose in field space when {@code CoordinateSpace.TAG} is requested
     * @return ordered list of target observations converted into the requested space
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
        Pose2d vendorPose = poseSupplier.get();
        Pose2d safePose = vendorPose != null ? vendorPose : new Pose2d();
        Pose3d vendorPose3d = pose3dSupplier.get();
        Pose3d safePose3d = vendorPose3d != null ? vendorPose3d : new Pose3d(safePose);
        LocalizationData data =
                new LocalizationData(safePose, safePose3d, latencySupplier.getAsDouble(), stdDevs);
        double defaultConfidence = getMeasurementConfidence();

        List<TargetObservation> observations = new ArrayList<>(measurements.size());
        for (TargetMeasurement measurement : measurements) {
            Optional<Translation2d> cameraTranslationOpt = cameraTranslationFromMeasurement(measurement);
            Optional<Translation2d> translatedOpt =
                    cameraTranslationOpt.flatMap(
                            ct ->
                                    convertCameraTranslationToSpace(
                                            ct, space, robotPose, vendorPose, tagPose));
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

    /**
     * Convenience overload of {@link #getObservations(CoordinateSpace, Pose2d, Pose2d)} that uses the
     * camera's current pose estimate when explicit poses are optional for the requested space.
     */
    public List<TargetObservation> getObservations(CoordinateSpace space) {
        return getObservations(space, null, null);
    }

    /**
     * Convenience overload of {@link #getObservations(CoordinateSpace, Pose2d, Pose2d)} for coordinate
     * systems that only require the robot pose (e.g. {@link CoordinateSpace#FIELD}).
     *
     * @param space target coordinate system
     * @param robotPose robot pose in field space when required
     */
    public List<TargetObservation> getObservations(CoordinateSpace space, Pose2d robotPose) {
        return getObservations(space, robotPose, null);
    }

    /**
     * Projects the latest target translation into the global field frame.
     *
     * @param robotPose current field-relative robot pose
     * @param cameraOffsetMeters override for the camera offset, or {@code null} to use the configured
     *     transform
     */
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

    /**
     * Estimates the field-relative robot pose using the supplied tag pose and current measurements.
     *
     * @param tagPose known pose of the observed tag in field space
     * @param cameraOffsetMeters override for the camera offset, or {@code null} to use the configured
     *     transform
     */
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
     * Returns the target translation expressed in the requested coordinate space.
     *
     * @param space coordinate space to convert into
     * @param robotPose robot pose in field space when required for the conversion
     * @param tagPose tag pose in field space when {@code CoordinateSpace.TAG} is requested
     */
    public Optional<Translation2d> getTargetTranslation(CoordinateSpace space, Pose2d robotPose, Pose2d tagPose) {
        List<TargetObservation> observations = getObservations(space, robotPose, tagPose);
        if (observations.isEmpty()) {
            return Optional.empty();
        }
        return Optional.ofNullable(observations.get(0).translation());
    }

    /**
     * Convenience overload of {@link #getTargetTranslation(CoordinateSpace, Pose2d, Pose2d)} that
     * relies solely on the data required for the chosen coordinate space, falling back to the camera's
     * estimated pose when appropriate.
     *
     * @param space target coordinate system
     */
    public Optional<Translation2d> getTargetTranslation(CoordinateSpace space) {
        return getTargetTranslation(space, null, null);
    }

    /**
     * Convenience overload of {@link #getTargetTranslation(CoordinateSpace, Pose2d, Pose2d)} that
     * accepts only the robot pose when that is the only additional information required for the
     * conversion.
     *
     * @param space target coordinate system
     * @param robotPose robot pose in field space when required
     */
    public Optional<Translation2d> getTargetTranslation(CoordinateSpace space, Pose2d robotPose) {
        return getTargetTranslation(space, robotPose, null);
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

    /**
     * Converts a camera-space translation into the requested coordinate system using the available
     * pose information.
     */
    private Optional<Translation2d> convertCameraTranslationToSpace(
            Translation2d cameraTranslation,
            CoordinateSpace space,
            Pose2d robotPose,
            Pose2d fallbackRobotPose,
            Pose2d tagPose) {
        return switch (space) {
            case CAMERA -> Optional.of(cameraTranslation);
            case ROBOT -> Optional.of(toRobotTranslation(cameraTranslation));
            case FIELD -> {
                Pose2d resolvedPose = robotPose != null ? robotPose : fallbackRobotPose;
                if (resolvedPose == null) {
                    yield Optional.empty();
                }
                Translation2d robotRelative = toRobotTranslation(cameraTranslation);
                yield Optional.of(
                        resolvedPose.getTranslation()
                                .plus(robotRelative.rotateBy(resolvedPose.getRotation())));
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

    /**
     * Applies the robot-to-camera transform to move a camera translation into robot space.
     */
    private Translation2d toRobotTranslation(Translation2d cameraTranslation) {
        return toRobotTranslation(cameraTranslation, robotToCameraTranslation);
    }

    /**
     * Applies the robot-to-camera transform using an optional offset override.
     */
    private Translation2d toRobotTranslation(Translation2d cameraTranslation, Translation2d offsetOverride) {
        Translation2d rotated = cameraTranslation.rotateBy(cameraYawRelativeToRobot);
        Translation2d cameraOffset = offsetOverride != null ? offsetOverride : robotToCameraTranslation;
        return rotated.plus(cameraOffset);
    }

    /**
     * Immutable bundle of localization data produced alongside camera measurements.
     *
     * @param pose2d camera supplied robot pose in field space
     * @param pose3d camera supplied robot pose in 3D field space
     * @param latency seconds of delay between photon capture and usable telemetry
     * @param stdDevs covariance diagonal representing the vendor estimate quality
     */
    public static record LocalizationData(Pose2d pose2d, Pose3d pose3d, double latency, Matrix<N3, N1> stdDevs) {
        public LocalizationData {
            pose2d = pose2d != null ? pose2d : new Pose2d();
            pose3d = pose3d != null ? pose3d : new Pose3d(pose2d);
        }

    }

    /**
     * Normalized representation of an observed target at a moment in time.
     *
     * @param tagId fiducial identifier, or {@code -1} when not reported
     * @param yawDegrees horizontal angle from the camera optical axis in degrees
     * @param distanceMeters direct camera-to-target distance in meters
     * @param translation translation expressed in the chosen {@link CoordinateSpace}
     * @param space coordinate system used for the translation
     * @param localizationData additional localization metadata that accompanied the measurement
     * @param confidence vendor supplied confidence score for this observation
     */
    public static record TargetObservation(int tagId, double yawDegrees, double distanceMeters,
                                           Translation2d translation, CoordinateSpace space,
                                           LocalizationData localizationData, double confidence) {
        /**
         * True when the observation includes a resolved distance measurement.
         */
        public boolean hasDistance() {
            return !Double.isNaN(distanceMeters);
        }

        /**
         * True when the observation includes a resolved yaw measurement.
         */
        public boolean hasYaw() {
            return !Double.isNaN(yawDegrees);
        }

        /**
         * True when the observation includes a translated position.
         */
        public boolean hasTranslation() {
            return translation != null;
        }

        /**
         * Checks whether the observation is expressed in the expected coordinate space.
         */
        public boolean isSpace(CoordinateSpace expectedSpace) {
            return space == expectedSpace;
        }

        /**
         * True when the vendor supplied a non-NaN confidence value.
         */
        public boolean hasConfidence() {
            return !Double.isNaN(confidence);
        }
    }

    /**
     * Capability that exposes vendor LED controls.
     */
    public interface LedControl {
        /**
         * Updates the LED mode on the camera.
         *
         * @param mode desired LED state
         */
        void setLedMode(LedMode mode);

        /**
         * Returns the currently configured LED mode.
         */
        LedMode getLedMode();
    }

    /**
     * Capability that exposes pipeline switching for cameras that support multiple configurations.
     */
    public interface PipelineControl {
        /**
         * Selects the active pipeline on the camera.
         *
         * @param pipeline vendor specific pipeline index
         */
        void setPipeline(int pipeline);

        /**
         * Returns the currently active vendor pipeline.
         */
        int getPipeline();
    }

    /**
     * Capability that exposes stream configuration options for cameras that support them.
     */
    public interface StreamControl {
        /**
         * Sets how the camera lays out multiple image streams.
         *
         * @param mode stream composition mode
         */
        void setStreamMode(StreamMode mode);

        /**
         * Returns the currently selected stream mode.
         */
        StreamMode getStreamMode();
    }

    /**
     * LED behavior modes supported by the vendor API.
     */
    public enum LedMode {
        PIPELINE,
        OFF,
        BLINK,
        ON
    }

    /**
     * Stream layout options supported by the vendor API.
     */
    public enum StreamMode {
        STANDARD,
        PIP_MAIN,
        PIP_SECONDARY
    }
}
