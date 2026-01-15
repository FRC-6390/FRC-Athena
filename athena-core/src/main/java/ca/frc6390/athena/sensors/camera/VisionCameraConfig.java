package ca.frc6390.athena.sensors.camera;

import java.util.EnumMap;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntFunction;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Builder-style configuration for {@link VisionCamera}. Mirrors the pattern used by
 * {@code MotorControllerConfig} by relying on suppliers/consumers so that vendor-specific camera
 * implementations can plug in custom behaviour without the wrapper knowing about it.
 */
public class VisionCameraConfig implements ConfigurableCamera {

    private final String table;
    private final CameraSoftware software;
    private BooleanSupplier connectedSupplier = () -> true;
    private BooleanSupplier hasTargetsSupplier = () -> false;
    private Supplier<Pose2d> poseSupplier = Pose2d::new;
    private Supplier<Pose3d> pose3dSupplier = Pose3d::new;
    private DoubleSupplier latencySupplier = () -> 0.0;
    private IntSupplier visibleTargetsSupplier = () -> 0;
    private DoubleSupplier averageDistanceSupplier = () -> Double.MAX_VALUE;
    private Consumer<Pose2d> orientationConsumer = pose -> {};
    private Supplier<Pose2d> orientationSupplier;
    private Runnable updateHook;

    private boolean useForLocalization = false;
    private double trustDistance = 2.0;
    private Matrix<N3, N1> singleStdDevs = VecBuilder.fill(0.35, 0.35, Math.toRadians(5));
    private Matrix<N3, N1> multiStdDevs = VecBuilder.fill(0.25, 0.25, Math.toRadians(3));
    private Transform3d cameraToRobotTransform = new Transform3d();
    private Transform3d robotToCameraTransform = new Transform3d();
    private DoubleSupplier confidenceSupplier = () -> 1.0;
    private EnumSet<CameraRole> roles = EnumSet.noneOf(CameraRole.class);

    private final EnumMap<VisionCameraCapability, Object> capabilities =
            new EnumMap<>(VisionCameraCapability.class);
    private DoubleSupplier targetYawSupplier = () -> Double.NaN;
    private DoubleSupplier targetPitchSupplier = () -> Double.NaN;
    private DoubleSupplier targetDistanceSupplier = () -> Double.NaN;
    private IntSupplier tagIdSupplier = () -> -1;
    private DoubleSupplier poseAmbiguitySupplier = () -> Double.NaN;
    private DoubleSupplier cameraPitchSupplier = () -> Double.NaN;
    private DoubleSupplier cameraRollSupplier = () -> Double.NaN;
    private Supplier<List<VisionCamera.TargetMeasurement>> targetMeasurementsSupplier = List::of;
    private AprilTagFieldLayout fieldLayout;
    private double displayHorizontalFovDeg = Double.NaN;
    private double displayRangeMeters = Double.NaN;
    private boolean publishPoseTopic;
    private IntFunction<Pose2d> tagPoseResolver = id -> null;
    private double maxLatencySeconds = Double.POSITIVE_INFINITY;
    private double stdDevConfidenceExponent = 0.0;
    private double stdDevLatencyWeight = 0.0;
    private double stdDevDistanceWeight = 0.0;
    private double stdDevTagCountWeight = 0.0;
    private double stdDevAmbiguityWeight = 0.0;
    private double stdDevPitchWeight = 0.0;
    private double stdDevRollWeight = 0.0;
    private double stdDevBaseScale = 1.0;
    private double stdDevMinScale = 0.2;
    private double visionWeightMultiplier = 1.0;

    /**
     * Creates a configuration bound to a specific NetworkTables table and camera software stack.
     *
     * @param table NetworkTables table name used by the camera
     * @param software software family (PhotonVision, Limelight, etc.)
     */
    public VisionCameraConfig(String table, CameraSoftware software) {
        this.table = table;
        this.software = software;
    }

    /**
     * Supplies a connectivity predicate so dashboards can reflect whether the camera is online.
     */
    public VisionCameraConfig setConnectedSupplier(BooleanSupplier supplier) {
        this.connectedSupplier = supplier != null ? supplier : () -> true;
        return this;
    }

    /**
     * Supplies a predicate that reports whether the camera currently sees viable targets.
     */
    public VisionCameraConfig setHasTargetsSupplier(BooleanSupplier supplier) {
        this.hasTargetsSupplier = supplier != null ? supplier : () -> false;
        return this;
    }

    /**
     * Supplies the 2D pose estimate reported by the camera.
     */
    public VisionCameraConfig setPoseSupplier(Supplier<Pose2d> supplier) {
        this.poseSupplier = supplier != null ? supplier : Pose2d::new;
        return this;
    }

    /**
     * Supplies the full 3D pose estimate reported by the camera.
     */
    public VisionCameraConfig setPose3dSupplier(Supplier<Pose3d> supplier) {
        this.pose3dSupplier = supplier != null ? supplier : Pose3d::new;
        return this;
    }

    /**
     * Supplies the latency (seconds) between image capture and pose estimation.
     */
    public VisionCameraConfig setLatencySupplier(DoubleSupplier supplier) {
        this.latencySupplier = supplier != null ? supplier : () -> 0.0;
        return this;
    }

    /**
     * Sets the maximum latency (seconds) to accept for localization measurements.
     * Use a non-positive value to disable gating.
     */
    public VisionCameraConfig setMaxLatencySeconds(double maxLatencySeconds) {
        this.maxLatencySeconds = maxLatencySeconds;
        return this;
    }

    /**
     * Scales pose std devs using the camera confidence (power curve). Zero disables scaling.
     */
    public VisionCameraConfig setStdDevConfidenceExponent(double exponent) {
        this.stdDevConfidenceExponent = exponent;
        return this;
    }

    /**
     * Scales pose std devs using (1 + weight * latencySeconds). Zero disables scaling.
     */
    public VisionCameraConfig setStdDevLatencyWeight(double weight) {
        this.stdDevLatencyWeight = weight;
        return this;
    }

    /**
     * Scales pose std devs using (1 + weight * distanceMeters). Zero disables scaling.
     */
    public VisionCameraConfig setStdDevDistanceWeight(double weight) {
        this.stdDevDistanceWeight = weight;
        return this;
    }

    /**
     * Scales pose std devs using (1 + weight * (visibleTargets - 1)) in the denominator. Zero disables scaling.
     */
    public VisionCameraConfig setStdDevTagCountWeight(double weight) {
        this.stdDevTagCountWeight = weight;
        return this;
    }

    /**
     * Scales pose std devs using (1 + weight * ambiguity). Zero disables scaling.
     */
    public VisionCameraConfig setStdDevAmbiguityWeight(double weight) {
        this.stdDevAmbiguityWeight = weight;
        return this;
    }

    /**
     * Scales pose std devs using (1 + weight * |pitchDegrees|). Zero disables scaling.
     */
    public VisionCameraConfig setStdDevPitchWeight(double weight) {
        this.stdDevPitchWeight = weight;
        return this;
    }

    /**
     * Scales pose std devs using (1 + weight * |rollDegrees|). Zero disables scaling.
     */
    public VisionCameraConfig setStdDevRollWeight(double weight) {
        this.stdDevRollWeight = weight;
        return this;
    }

    /**
     * Scales pose std devs using a constant multiplier for this camera. One disables scaling.
     */
    public VisionCameraConfig setStdDevBaseScale(double scale) {
        this.stdDevBaseScale = scale;
        return this;
    }

    /**
     * Sets the minimum scale applied to std devs after dynamic adjustments.
     */
    public VisionCameraConfig setStdDevMinScale(double minScale) {
        this.stdDevMinScale = minScale;
        return this;
    }

    /**
     * Sets a per-camera weight multiplier applied during multi-camera fusion.
     */
    public VisionCameraConfig setVisionWeightMultiplier(double weight) {
        this.visionWeightMultiplier = weight;
        return this;
    }

    /**
     * Supplies the number of fiducial targets currently visible.
     */
    public VisionCameraConfig setVisibleTargetsSupplier(IntSupplier supplier) {
        this.visibleTargetsSupplier = supplier != null ? supplier : () -> 0;
        return this;
    }

    /**
     * Supplies the average distance (meters) from the camera to all detected targets.
     */
    public VisionCameraConfig setAverageDistanceSupplier(DoubleSupplier supplier) {
        this.averageDistanceSupplier = supplier != null ? supplier : () -> Double.MAX_VALUE;
        return this;
    }

    /**
     * Supplies a resolver that returns the field-relative pose for the provided tag identifier.
     *
     * @param resolver callback mapping fiducial IDs to poses, or {@code null} to disable auto lookup
     */
    public VisionCameraConfig setTagPoseResolver(IntFunction<Pose2d> resolver) {
        this.tagPoseResolver = resolver;
        return this;
    }

    /**
     * Installs a resolver that looks up tag poses from the supplied {@link AprilTagFieldLayout}.
     *
     * @param layout field layout used to translate tag identifiers into poses
     */
    public VisionCameraConfig setFieldLayout(AprilTagFieldLayout layout) {
        this.fieldLayout = layout;
        if (layout == null) {
            this.tagPoseResolver = id -> null;
        } else {
            this.tagPoseResolver = id -> layout.getTagPose(id).map(Pose3d::toPose2d).orElse(null);
        }
        return this;
    }

    /**
     * Provides a callback that accepts a new robot orientation when the camera pipeline refines it.
     */
    public VisionCameraConfig setOrientationConsumer(Consumer<Pose2d> consumer) {
        this.orientationConsumer = consumer != null ? consumer : pose -> {};
        return this;
    }

    /**
     * Supplies an alternate orientation estimate that can be merged with the camera pose.
     */
    public VisionCameraConfig setOrientationSupplier(Supplier<Pose2d> supplier) {
        this.orientationSupplier = supplier;
        return this;
    }

    /**
     * Registers a hook that runs after the camera updates its internal data structures.
     */
    public VisionCameraConfig setUpdateHook(Runnable hook) {
        this.updateHook = hook;
        return this;
    }

    /**
     * Flags whether this camera should contribute to robot pose estimation.
     */
    public VisionCameraConfig setUseForLocalization(boolean useForLocalization) {
        this.useForLocalization = useForLocalization;
        return this;
    }

    /**
     * Sets the maximum distance (meters) at which vision measurements are trusted without scaling
     * their uncertainty.
     */
    public VisionCameraConfig setTrustDistance(double trustDistance) {
        this.trustDistance = trustDistance;
        if (Double.isNaN(displayRangeMeters) || displayRangeMeters <= 0.0) {
            displayRangeMeters = trustDistance;
        }
        return this;
    }

    /**
     * Supplies the single-tag measurement standard deviations (x, y, theta).
     */
    public VisionCameraConfig setSingleStdDevs(Matrix<N3, N1> singleStdDevs) {
        this.singleStdDevs = singleStdDevs != null ? singleStdDevs : this.singleStdDevs;
        return this;
    }

    /**
     * Supplies the multi-tag measurement standard deviations (x, y, theta).
     */
    public VisionCameraConfig setMultiStdDevs(Matrix<N3, N1> multiStdDevs) {
        this.multiStdDevs = multiStdDevs != null ? multiStdDevs : this.multiStdDevs;
        return this;
    }

    /**
     * Sets the camera-to-robot translation using a planar offset (meters). Rotation and height
     * components remain unchanged.
     */
    public VisionCameraConfig setCameraToRobotTranslation(Translation2d translation) {
        Translation2d value = translation != null ? translation : new Translation2d();
        robotToCameraTransform = new Transform3d(
                value.getX(),
                value.getY(),
                robotToCameraTransform.getZ(),
                robotToCameraTransform.getRotation());
        cameraToRobotTransform = robotToCameraTransform.inverse();
        return this;
    }

    /**
     * Sets the transform from robot origin to camera origin directly.
     */
    public VisionCameraConfig setRobotToCameraTransform(Transform3d transform) {
        Transform3d value = transform != null ? transform : new Transform3d();
        this.robotToCameraTransform = value;
        this.cameraToRobotTransform = value.inverse();
        return this;
    }

    /**
     * Sets the transform from camera origin to robot origin directly.
     */
    public VisionCameraConfig setCameraToRobotTransform(Transform3d transform) {
        Transform3d value = transform != null ? transform : new Transform3d();
        this.cameraToRobotTransform = value;
        this.robotToCameraTransform = value.inverse();
        return this;
    }

    /**
     * Supplies the yaw (horizontal offset) to the best target in degrees.
     */
    public VisionCameraConfig setTargetYawSupplier(DoubleSupplier supplier) {
        this.targetYawSupplier = supplier != null ? supplier : () -> Double.NaN;
        return this;
    }

    /**
     * Supplies the pitch (vertical offset) to the best target in degrees.
     */
    public VisionCameraConfig setTargetPitchSupplier(DoubleSupplier supplier) {
        this.targetPitchSupplier = supplier != null ? supplier : () -> Double.NaN;
        return this;
    }

    /**
     * Supplies the pose ambiguity reported by the vision pipeline (0-1, higher is worse).
     */
    public VisionCameraConfig setPoseAmbiguitySupplier(DoubleSupplier supplier) {
        this.poseAmbiguitySupplier = supplier != null ? supplier : () -> Double.NaN;
        return this;
    }

    /**
     * Supplies the camera pitch in degrees.
     */
    public VisionCameraConfig setCameraPitchSupplier(DoubleSupplier supplier) {
        this.cameraPitchSupplier = supplier != null ? supplier : () -> Double.NaN;
        return this;
    }

    /**
     * Supplies the camera roll in degrees.
     */
    public VisionCameraConfig setCameraRollSupplier(DoubleSupplier supplier) {
        this.cameraRollSupplier = supplier != null ? supplier : () -> Double.NaN;
        return this;
    }

    /**
     * Supplies the overall confidence value [0, 1] returned by the camera pipeline.
     */
    public VisionCameraConfig setConfidenceSupplier(DoubleSupplier supplier) {
        this.confidenceSupplier = supplier != null ? supplier : () -> 1.0;
        return this;
    }

    /**
     * Adds an operational role for the camera (aiming, localization, driver camera, etc.).
     */
    public VisionCameraConfig addRole(CameraRole role) {
        if (role != null) {
            roles.add(role);
        }
        return this;
    }

    /**
     * Replaces the camera's role set entirely.
     */
    public VisionCameraConfig setRoles(EnumSet<CameraRole> roles) {
        this.roles = roles != null ? EnumSet.copyOf(roles) : EnumSet.noneOf(CameraRole.class);
        return this;
    }

    /**
     * Supplies the distance (meters) to the best tag, if available.
     */
    public VisionCameraConfig setTagDistanceSupplier(DoubleSupplier supplier) {
        this.targetDistanceSupplier = supplier != null ? supplier : () -> Double.NaN;
        return this;
    }

    /**
     * Supplies the ID of the best tag being tracked (-1 when none).
     */
    public VisionCameraConfig setTagIdSupplier(IntSupplier supplier) {
        this.tagIdSupplier = supplier != null ? supplier : () -> -1;
        return this;
    }

    /**
     * Supplies the full list of per-target measurements returned by the camera.
     */
    public VisionCameraConfig setTargetMeasurementsSupplier(
            Supplier<List<VisionCamera.TargetMeasurement>> supplier) {
        this.targetMeasurementsSupplier = supplier != null ? supplier : List::of;
        return this;
    }

    /**
     * Sets the horizontal field of view (degrees) used by dashboards to scale overlays.
     */
    public VisionCameraConfig setDisplayHorizontalFov(double degrees) {
        this.displayHorizontalFovDeg = degrees;
        return this;
    }

    /**
     * Sets the render range (meters) used by dashboards for front-panel visualization.
     */
    public VisionCameraConfig setDisplayRangeMeters(double rangeMeters) {
        this.displayRangeMeters = rangeMeters;
        return this;
    }

    /**
     * Enables publishing an estimated pose topic for dashboards. Disabled by default to avoid
     * creating stray NetworkTables paths.
     */
    public VisionCameraConfig setPublishPoseTopic(boolean enabled) {
        this.publishPoseTopic = enabled;
        return this;
    }

    public boolean isPublishPoseTopicEnabled() {
        return publishPoseTopic;
    }

    /**
     * Registers a camera-specific capability object that can be queried later by the wrapper.
     *
     * @throws IllegalArgumentException if the capability instance does not match the expected type
     */
    public VisionCameraConfig addCapability(VisionCameraCapability capabilityKey, Object capability) {
        if (capabilityKey != null && capability != null) {
            if (!capabilityKey.getType().isInstance(capability)) {
                throw new IllegalArgumentException(
                        "Capability " + capabilityKey + " expects implementation of type "
                                + capabilityKey.getType().getName()
                                + " but received "
                                + capability.getClass().getName());
            }
            capabilities.put(capabilityKey, capability);
        }
        return this;
    }

    public BooleanSupplier getConnectedSupplier() {
        return connectedSupplier;
    }

    public BooleanSupplier getHasTargetsSupplier() {
        return hasTargetsSupplier;
    }

    public Supplier<Pose2d> getPoseSupplier() {
        return poseSupplier;
    }

    public Supplier<Pose3d> getPose3dSupplier() {
        return pose3dSupplier;
    }

    public DoubleSupplier getLatencySupplier() {
        return latencySupplier;
    }

    public IntSupplier getVisibleTargetsSupplier() {
        return visibleTargetsSupplier;
    }

    public DoubleSupplier getAverageDistanceSupplier() {
        return averageDistanceSupplier;
    }

    public IntFunction<Pose2d> getTagPoseResolver() {
        return tagPoseResolver;
    }

    public Consumer<Pose2d> getOrientationConsumer() {
        return orientationConsumer;
    }

    public Supplier<Pose2d> getOrientationSupplier() {
        return orientationSupplier;
    }

    public Runnable getUpdateHook() {
        return updateHook;
    }

    public boolean isUseForLocalization() {
        return useForLocalization;
    }

    public double getTrustDistance() {
        return trustDistance;
    }

    public Matrix<N3, N1> getSingleStdDevs() {
        return singleStdDevs;
    }

    public Matrix<N3, N1> getMultiTagStdDevs() {
        return multiStdDevs;
    }

    public Translation2d getCameraToRobotTranslation() {
        return new Translation2d(cameraToRobotTransform.getX(), cameraToRobotTransform.getY());
    }

    public Transform3d getCameraToRobotTransform() {
        return cameraToRobotTransform;
    }

    public Transform3d getRobotToCameraTransform() {
        return robotToCameraTransform;
    }

    public DoubleSupplier getTargetYawSupplier() {
        return targetYawSupplier;
    }

    public DoubleSupplier getTargetPitchSupplier() {
        return targetPitchSupplier;
    }

    public DoubleSupplier getTagDistanceSupplier() {
        return targetDistanceSupplier;
    }

    public IntSupplier getTagIdSupplier() {
        return tagIdSupplier;
    }

    public DoubleSupplier getPoseAmbiguitySupplier() {
        return poseAmbiguitySupplier;
    }

    public DoubleSupplier getCameraPitchSupplier() {
        return cameraPitchSupplier;
    }

    public DoubleSupplier getCameraRollSupplier() {
        return cameraRollSupplier;
    }

    public Supplier<List<VisionCamera.TargetMeasurement>> getTargetMeasurementsSupplier() {
        return targetMeasurementsSupplier;
    }

    public AprilTagFieldLayout getFieldLayout() {
        return fieldLayout;
    }

    public double getDisplayHorizontalFov() {
        return displayHorizontalFovDeg;
    }

    public double getDisplayRangeMeters() {
        return displayRangeMeters;
    }

    public DoubleSupplier getConfidenceSupplier() {
        return confidenceSupplier;
    }

    public double getMaxLatencySeconds() {
        return maxLatencySeconds;
    }

    public double getStdDevConfidenceExponent() {
        return stdDevConfidenceExponent;
    }

    public double getStdDevLatencyWeight() {
        return stdDevLatencyWeight;
    }

    public double getStdDevDistanceWeight() {
        return stdDevDistanceWeight;
    }

    public double getStdDevTagCountWeight() {
        return stdDevTagCountWeight;
    }

    public double getStdDevAmbiguityWeight() {
        return stdDevAmbiguityWeight;
    }

    public double getStdDevPitchWeight() {
        return stdDevPitchWeight;
    }

    public double getStdDevRollWeight() {
        return stdDevRollWeight;
    }

    public double getStdDevBaseScale() {
        return stdDevBaseScale;
    }

    public double getStdDevMinScale() {
        return stdDevMinScale;
    }

    public double getVisionWeightMultiplier() {
        return visionWeightMultiplier;
    }

    public EnumSet<CameraRole> getRoles() {
        return roles.isEmpty() ? EnumSet.noneOf(CameraRole.class) : EnumSet.copyOf(roles);
    }

    public Map<VisionCameraCapability, Object> capabilities() {
        return capabilities;
    }

    @Override
    public CameraSoftware getSoftware() {
        return software;
    }

    @Override
    public String getTable() {
        return table;
    }

    public enum CameraRole {
        LOCALIZATION,
        SCORING,
        DRIVER,
        AUTO,
        CUSTOM
    }
}
