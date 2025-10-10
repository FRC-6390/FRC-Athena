package ca.frc6390.athena.sensors.camera;

import java.util.EnumMap;
import java.util.EnumSet;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.IntSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

/**
 * Builder-style configuration for {@link LocalizationCamera}. Mirrors the pattern used by
 * {@code MotorControllerConfig} by relying on suppliers/consumers so that vendor-specific camera
 * implementations can plug in custom behaviour without the wrapper knowing about it.
 */
public class LocalizationCameraConfig implements ConfigurableCamera {

    private final String table;
    private final CameraSoftware software;
    private BooleanSupplier connectedSupplier = () -> true;
    private BooleanSupplier hasTargetsSupplier = () -> false;
    private Supplier<Pose2d> poseSupplier = Pose2d::new;
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

    private final EnumMap<LocalizationCameraCapability, Object> capabilities =
            new EnumMap<>(LocalizationCameraCapability.class);
    private DoubleSupplier targetYawSupplier = () -> Double.NaN;
    private DoubleSupplier targetPitchSupplier = () -> Double.NaN;
    private DoubleSupplier targetDistanceSupplier = () -> Double.NaN;
    private IntSupplier tagIdSupplier = () -> -1;
    private Supplier<List<LocalizationCamera.TargetMeasurement>> targetMeasurementsSupplier = List::of;
    private double displayHorizontalFovDeg = Double.NaN;
    private double displayRangeMeters = Double.NaN;

    public LocalizationCameraConfig(String table, CameraSoftware software) {
        this.table = table;
        this.software = software;
    }

    public LocalizationCameraConfig setConnectedSupplier(BooleanSupplier supplier) {
        this.connectedSupplier = supplier != null ? supplier : () -> true;
        return this;
    }

    public LocalizationCameraConfig setHasTargetsSupplier(BooleanSupplier supplier) {
        this.hasTargetsSupplier = supplier != null ? supplier : () -> false;
        return this;
    }

    public LocalizationCameraConfig setPoseSupplier(Supplier<Pose2d> supplier) {
        this.poseSupplier = supplier != null ? supplier : Pose2d::new;
        return this;
    }

    public LocalizationCameraConfig setLatencySupplier(DoubleSupplier supplier) {
        this.latencySupplier = supplier != null ? supplier : () -> 0.0;
        return this;
    }

    public LocalizationCameraConfig setVisibleTargetsSupplier(IntSupplier supplier) {
        this.visibleTargetsSupplier = supplier != null ? supplier : () -> 0;
        return this;
    }

    public LocalizationCameraConfig setAverageDistanceSupplier(DoubleSupplier supplier) {
        this.averageDistanceSupplier = supplier != null ? supplier : () -> Double.MAX_VALUE;
        return this;
    }

    public LocalizationCameraConfig setOrientationConsumer(Consumer<Pose2d> consumer) {
        this.orientationConsumer = consumer != null ? consumer : pose -> {};
        return this;
    }

    public LocalizationCameraConfig setOrientationSupplier(Supplier<Pose2d> supplier) {
        this.orientationSupplier = supplier;
        return this;
    }

    public LocalizationCameraConfig setUpdateHook(Runnable hook) {
        this.updateHook = hook;
        return this;
    }

    public LocalizationCameraConfig setUseForLocalization(boolean useForLocalization) {
        this.useForLocalization = useForLocalization;
        return this;
    }

    public LocalizationCameraConfig setTrustDistance(double trustDistance) {
        this.trustDistance = trustDistance;
        if (Double.isNaN(displayRangeMeters) || displayRangeMeters <= 0.0) {
            displayRangeMeters = trustDistance;
        }
        return this;
    }

    public LocalizationCameraConfig setSingleStdDevs(Matrix<N3, N1> singleStdDevs) {
        this.singleStdDevs = singleStdDevs != null ? singleStdDevs : this.singleStdDevs;
        return this;
    }

    public LocalizationCameraConfig setMultiStdDevs(Matrix<N3, N1> multiStdDevs) {
        this.multiStdDevs = multiStdDevs != null ? multiStdDevs : this.multiStdDevs;
        return this;
    }

    public LocalizationCameraConfig setCameraToRobotTranslation(Translation2d translation) {
        Translation2d value = translation != null ? translation : new Translation2d();
        robotToCameraTransform = new Transform3d(
                value.getX(),
                value.getY(),
                robotToCameraTransform.getZ(),
                robotToCameraTransform.getRotation());
        cameraToRobotTransform = robotToCameraTransform.inverse();
        return this;
    }

    public LocalizationCameraConfig setRobotToCameraTransform(Transform3d transform) {
        Transform3d value = transform != null ? transform : new Transform3d();
        this.robotToCameraTransform = value;
        this.cameraToRobotTransform = value.inverse();
        return this;
    }

    public LocalizationCameraConfig setCameraToRobotTransform(Transform3d transform) {
        Transform3d value = transform != null ? transform : new Transform3d();
        this.cameraToRobotTransform = value;
        this.robotToCameraTransform = value.inverse();
        return this;
    }

    public LocalizationCameraConfig setTargetYawSupplier(DoubleSupplier supplier) {
        this.targetYawSupplier = supplier != null ? supplier : () -> Double.NaN;
        return this;
    }

    public LocalizationCameraConfig setTargetPitchSupplier(DoubleSupplier supplier) {
        this.targetPitchSupplier = supplier != null ? supplier : () -> Double.NaN;
        return this;
    }

    public LocalizationCameraConfig setConfidenceSupplier(DoubleSupplier supplier) {
        this.confidenceSupplier = supplier != null ? supplier : () -> 1.0;
        return this;
    }

    public LocalizationCameraConfig addRole(CameraRole role) {
        if (role != null) {
            roles.add(role);
        }
        return this;
    }

    public LocalizationCameraConfig setRoles(EnumSet<CameraRole> roles) {
        this.roles = roles != null ? EnumSet.copyOf(roles) : EnumSet.noneOf(CameraRole.class);
        return this;
    }

    public LocalizationCameraConfig setTagDistanceSupplier(DoubleSupplier supplier) {
        this.targetDistanceSupplier = supplier != null ? supplier : () -> Double.NaN;
        return this;
    }

    public LocalizationCameraConfig setTagIdSupplier(IntSupplier supplier) {
        this.tagIdSupplier = supplier != null ? supplier : () -> -1;
        return this;
    }

    public LocalizationCameraConfig setTargetMeasurementsSupplier(
            Supplier<List<LocalizationCamera.TargetMeasurement>> supplier) {
        this.targetMeasurementsSupplier = supplier != null ? supplier : List::of;
        return this;
    }

    public LocalizationCameraConfig setDisplayHorizontalFov(double degrees) {
        this.displayHorizontalFovDeg = degrees;
        return this;
    }

    public LocalizationCameraConfig setDisplayRangeMeters(double rangeMeters) {
        this.displayRangeMeters = rangeMeters;
        return this;
    }

    public LocalizationCameraConfig addCapability(LocalizationCameraCapability capabilityKey, Object capability) {
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

    public DoubleSupplier getLatencySupplier() {
        return latencySupplier;
    }

    public IntSupplier getVisibleTargetsSupplier() {
        return visibleTargetsSupplier;
    }

    public DoubleSupplier getAverageDistanceSupplier() {
        return averageDistanceSupplier;
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

    public Supplier<List<LocalizationCamera.TargetMeasurement>> getTargetMeasurementsSupplier() {
        return targetMeasurementsSupplier;
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

    public EnumSet<CameraRole> getRoles() {
        return roles.isEmpty() ? EnumSet.noneOf(CameraRole.class) : EnumSet.copyOf(roles);
    }

    public Map<LocalizationCameraCapability, Object> capabilities() {
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
