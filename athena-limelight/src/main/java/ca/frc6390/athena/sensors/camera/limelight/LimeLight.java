package ca.frc6390.athena.sensors.camera.limelight;

import java.util.Arrays;
import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.OptionalDouble;
import java.util.OptionalInt;
import java.util.Set;

import ca.frc6390.athena.sensors.camera.VisionCamera;
import ca.frc6390.athena.sensors.camera.VisionCamera.PipelineControl;
import ca.frc6390.athena.sensors.camera.VisionCamera.StreamControl;
import ca.frc6390.athena.sensors.camera.VisionCameraConfig;
import ca.frc6390.athena.sensors.camera.VisionCameraConfig.CameraRole;
import ca.frc6390.athena.sensors.camera.VisionCameraCapability;
import ca.frc6390.athena.sensors.camera.FiducialSource;
import ca.frc6390.athena.sensors.camera.LimelightCamera;
import ca.frc6390.athena.sensors.camera.LocalizationSource;
import ca.frc6390.athena.sensors.camera.TargetingSource;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;

public class LimeLight implements LimelightCamera, LocalizationSource, TargetingSource, FiducialSource {
    public LimeLightConfig config;
    private NetworkTable limelightTable;
    public NetworkTableEntry tv;
    public NetworkTableEntry tx;
    public NetworkTableEntry ty;
    public NetworkTableEntry txnc;
    public NetworkTableEntry tync;
    public NetworkTableEntry botpose_targetspace_set;
    public NetworkTableEntry ta;
    public NetworkTableEntry crosshairs;
    public NetworkTableEntry t2d;
    public NetworkTableEntry tl;
    public NetworkTableEntry tshort;
    public NetworkTableEntry tlong;
    public NetworkTableEntry thor;
    public NetworkTableEntry getpipe;
    public NetworkTableEntry camtran;
    public NetworkTableEntry tid;
    public NetworkTableEntry json;
    public NetworkTableEntry camerapose_robotspace_set;
    public NetworkTableEntry robot_orientation_set;
    public NetworkTableEntry fiducial_id_filters_set;
    public NetworkTableEntry fiducial_offset_set;
    public NetworkTableEntry tclass;
    public NetworkTableEntry priorityid;
    public NetworkTableEntry tc;
    public NetworkTableEntry ledMode;
    public NetworkTableEntry camMode;
    public NetworkTableEntry pipeline;
    public NetworkTableEntry stream;
    public NetworkTableEntry snapshot;
    public NetworkTableEntry crop;
    public NetworkTableEntry tx0;
    public NetworkTableEntry ty0;
    public NetworkTableEntry ta0;
    public NetworkTableEntry ts0;
    public NetworkTableEntry tx1;
    public NetworkTableEntry ty1;
    public NetworkTableEntry ta1;
    public NetworkTableEntry ts1;
    public NetworkTableEntry tx2;
    public NetworkTableEntry ty2;
    public NetworkTableEntry ta2;
    public NetworkTableEntry ts2;
    public NetworkTableEntry cx0;
    public NetworkTableEntry cy0;
    public NetworkTableEntry cx1;
    public NetworkTableEntry cy1;
    public NetworkTableEntry rawfiducials;
    public boolean useForLocalization = false;

    private final VisionCamera localizationCamera;
    private final AprilTagFieldLayout fieldLayout;
    private LocalizationSnapshot latestSnapshot = LocalizationSnapshot.empty();
    private static final double DEFAULT_HORIZONTAL_FOV_DEG = 63.3;

    private static final class LocalizationSnapshot {
        private final Pose2d pose;
        private final Pose3d pose3d;
        private final double latencySeconds;
        private final int visibleTargets;
        private final double averageDistanceMeters;

        private LocalizationSnapshot(
                Pose2d pose, Pose3d pose3d, double latencySeconds, int visibleTargets, double averageDistanceMeters) {
            this.pose = pose;
            this.pose3d = pose3d;
            this.latencySeconds = latencySeconds;
            this.visibleTargets = visibleTargets;
            this.averageDistanceMeters = averageDistanceMeters;
        }

        public static LocalizationSnapshot empty() {
            return new LocalizationSnapshot(new Pose2d(), new Pose3d(), 0.0, 0, Double.MAX_VALUE);
        }

        public Pose2d pose() {
            return pose;
        }

        public Pose3d pose3d() {
            return pose3d;
        }

        public double latencySeconds() {
            return latencySeconds;
        }

        public int visibleTargets() {
            return visibleTargets;
        }

        public double averageDistanceMeters() {
            return averageDistanceMeters;
        }
    }

    public enum LedMode{
        PIPELINE(0),
        OFF(1),
        BLINK(2),
        ON(3);
        private int id;
        private LedMode(int id){
            this.id = id;
        }

        public int get(){
            return id;
        }
    }

    public enum PoseEstimateWithLatencyType{
        BOT_POSE("botpose"), 
        BOT_POSE_RED("botpose_wpired"),
        BOT_POSE_BLUE("botpose_wpiblue"),
        BOT_POSE_MT2_BLUE("botpose_orb_wpiblue"),
        BOT_POSE_MT2_RED("botpose_orb_wpired"),
        BOT_POSE_MT2("botpose_orb");
        private String id;
        private PoseEstimateWithLatencyType(String id){
            this.id = id;
        }

        public String get(){
            return id;
        }
    }

    public enum PoseEstimateType{
        CAMERA_POSE_TARGET_SPACE("camerapose_targetspace"),
        TARGET_POSE_CAMERA_SPACE("targetpose_cameraspace"),
        TARGET_POSE_ROBOT_SPACE("targetpose_robotspace"),
        BOT_POSE_TARGET_SPACE("botpose_targetspace"),
        CAMERA_POSE_ROBOT_SPACE("camerapose_robotspace");
        private String id;
        private PoseEstimateType(String id){
            this.id = id;
        }

        public String get(){
            return id;
        }
    }
    


    public enum CameraMode{
        VISION_PROCESSOR(0),
        DRIVER(1);
        private int id;
        private CameraMode(int id){
            this.id = id;
        }

        public int get(){
            return id;
        }
    }

    public enum StreamMode{
        STANDARD(0),
        PIP_MAIN(1),
        PIP_SECONDARY(2);
        private int id;
        private StreamMode(int id){
            this.id = id;
        }

        public int get(){
            return id;
        }
    }

    public enum SnapshotMode{
        RESET(0),
        ONCE(1);
        private int id;
        private SnapshotMode(int id){
            this.id = id;
        }

        public int get(){
            return id;
        }
    }

    public class PoseEstimateWithLatency extends PoseEstimate
    {
        
        public PoseEstimateWithLatency(PoseEstimateWithLatencyType type)
        {
            super(type.get());
        }

        private double getRawValue(int idx) {
            Double[] raw = getRaw();
            if (raw == null || idx < 0 || idx >= raw.length) {
                return Double.NaN;
            }
            Double value = raw[idx];
            return value != null ? value.doubleValue() : Double.NaN;
        }

        /**
         * Limelight "botpose*" arrays:
         * - Older format: [0..5]=pose, [6]=latency(ms), [7]=tagCount, [8]=avgTagDist(m), [9]=avgTagArea(%)
         * - Newer format: [0..5]=pose, [6]=latency(ms), [7]=tagCount, [8]=tagSpan, [9]=avgTagDist(m), [10]=avgTagArea(%)
         */
        public double getLatency() {
            return getRawValue(6);
        }

        public int getTagCount() {
            double value = getRawValue(7);
            return Double.isFinite(value) ? (int) value : 0;
        }

        /**
         * Average distance from camera to observed tags, in meters.
         *
         * Note: newer Limelight arrays include a tagSpan element; in that case avgTagDist shifts
         * from index 8 to index 9. Returning index 8 in the newer format often yields 0 for a
         * single-tag observation (tagSpan = 0).
         */
        public double getDistToTag() {
            Double[] raw = getRaw();
            if (raw == null) {
                return Double.NaN;
            }
            if (raw.length >= 11) {
                return getRawValue(9);
            }
            if (raw.length >= 10) {
                return getRawValue(8);
            }
            return Double.NaN;
        }

        public double getAvgTagArea() {
            Double[] raw = getRaw();
            if (raw == null) {
                return Double.NaN;
            }
            if (raw.length >= 11) {
                return getRawValue(10);
            }
            if (raw.length >= 10) {
                return getRawValue(9);
            }
            return Double.NaN;
        }
        
    }

    public class PoseEstimate
    {
        // Default to NaN so callers can distinguish "no data" from a legitimate 0.
        // We use 11 elements to match Limelight's extended botpose* arrays.
        Double[] dub = new Double[]{
                Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN,
                Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN
        };
        String table;

        public PoseEstimate(PoseEstimateType type)
        {
           this(type.get());
        }

        public PoseEstimate(String table)
        {
            this.table = table;
        }
        public Double[] getRaw()
        {
            return limelightTable.getEntry(table).getDoubleArray(dub);
        }

        public Pose2d getPose()
        {
            Double[] poseReal = getRaw();
            if (poseReal == null) return new Pose2d();
            if (poseReal.length < 6
                    || !Double.isFinite(poseReal[0])
                    || !Double.isFinite(poseReal[2])
                    || !Double.isFinite(poseReal[5])) {
                return new Pose2d();
            }
            Translation2d translation = new Translation2d(poseReal[2], poseReal[0]);
            Rotation2d rotation2d = Rotation2d.fromDegrees(poseReal[5]);
            return new Pose2d(translation, rotation2d);
        }

        public Pose2d getLocalizationPose()
        {
            Double[] poseReal = getRaw();
            if (poseReal == null) return new Pose2d();
            if (poseReal.length < 6
                    || !Double.isFinite(poseReal[0])
                    || !Double.isFinite(poseReal[1])
                    || !Double.isFinite(poseReal[5])) {
                return new Pose2d();
            }
            Translation2d translation = new Translation2d(poseReal[0], poseReal[1]);
            Rotation2d rotation2d = Rotation2d.fromDegrees(poseReal[5]);
            return new Pose2d(translation, rotation2d);
        }
    }

    public LimeLight(LimeLightConfig config){
        this.config = config;
        AprilTagFieldLayout layout;
        try {
            layout = AprilTagFieldLayout.loadField(config.fieldLayout());
        } catch (Exception e) {
            layout = null;
        }
        this.fieldLayout = layout;
        limelightTable = NetworkTableInstance.getDefault().getTable(config.table());
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        txnc = limelightTable.getEntry("txnc");
        tync = limelightTable.getEntry("tync");
        ta = limelightTable.getEntry("ta");
        crosshairs = limelightTable.getEntry("crosshairs");
        t2d = limelightTable.getEntry("t2d");
        tl = limelightTable.getEntry("tl");
        tshort = limelightTable.getEntry("tshort");
        tlong = limelightTable.getEntry("tlong");
        thor = limelightTable.getEntry("thor");
        getpipe = limelightTable.getEntry("getpipe");
        camtran = limelightTable.getEntry("camtran");
        tid = limelightTable.getEntry("tid");
        json = limelightTable.getEntry("json");
       
        tclass = limelightTable.getEntry("tclass");
        priorityid = limelightTable.getEntry("priorityid");
      
        botpose_targetspace_set = limelightTable.getEntry("botpose_targetspace_set");
        camerapose_robotspace_set = limelightTable.getEntry("camerapose_robotspace_set");
        robot_orientation_set = limelightTable.getEntry("robot_orientation_set");
        fiducial_id_filters_set = limelightTable.getEntry("fiducial_id_filters_set");
        fiducial_offset_set = limelightTable.getEntry("fiducial_offset_set");
        
        tc = limelightTable.getEntry("tc");
        ledMode = limelightTable.getEntry("ledMode");
        camMode = limelightTable.getEntry("camMode");
        pipeline = limelightTable.getEntry("pipeline");
        stream = limelightTable.getEntry("stream");
        snapshot = limelightTable.getEntry("snapshot");
        crop = limelightTable.getEntry("crop");
        tx0 = limelightTable.getEntry("tx0");
        ty0 = limelightTable.getEntry("ty0");
        ta0 = limelightTable.getEntry("ta0");
        ts0 = limelightTable.getEntry("ts0");
        tx1 = limelightTable.getEntry("tx1");
        ty1 = limelightTable.getEntry("ty1");
        ta1 = limelightTable.getEntry("ta1");
        ts1 = limelightTable.getEntry("ts1");
        tx2 = limelightTable.getEntry("tx2");
        ty2 = limelightTable.getEntry("ty2");
        ta2 = limelightTable.getEntry("ta2");
        ts2 = limelightTable.getEntry("ts2");
        cx0 = limelightTable.getEntry("cx0");
        cy0 = limelightTable.getEntry("cy0");
        cx1 = limelightTable.getEntry("cx1");
        cy1 = limelightTable.getEntry("cy1");
        rawfiducials =  limelightTable.getEntry("rawfiducials");
        useForLocalization = config.useForLocalization();
        applyLocalizationTagFilters();
        applyCameraTransformToNetworkTables(config.cameraRobotSpace());
        refreshLocalizationSnapshot();
        localizationCamera = createVisionCamera();
    }

    private void applyLocalizationTagFilters() {
        int[] filteredTags = config.filteredTags();
        if (filteredTags == null || filteredTags.length == 0) {
            setFiducialIdFilters(new double[] {});
            return;
        }

        if (fieldLayout == null) {
            setFiducialIdFilters(Arrays.stream(filteredTags).mapToDouble(i -> i).toArray());
            return;
        }

        List<AprilTag> tags = fieldLayout.getTags();
        if (tags == null || tags.isEmpty()) {
            setFiducialIdFilters(Arrays.stream(filteredTags).mapToDouble(i -> i).toArray());
            return;
        }

        Set<Integer> filteredSet = new HashSet<>();
        for (int tagId : filteredTags) {
            filteredSet.add(tagId);
        }

        double[] allowedTags = tags.stream()
                .mapToInt(tag -> tag.ID)
                .filter(id -> !filteredSet.contains(id))
                .distinct()
                .mapToDouble(id -> id)
                .toArray();
        setFiducialIdFilters(allowedTags);
    }

    private VisionCamera createVisionCamera() {
        VisionCameraConfig cameraConfig =
                new VisionCameraConfig(config.getTable(), config.getSoftware())
                        .setUseForLocalization(config.useForLocalization())
                        .setTrustDistance(config.trustDistance())
                        .setHasTargetsSupplier(this::hasValidTarget)
                        .setPoseSupplier(this::supplyLocalizationPose)
                        .setPose3dSupplier(this::supplyLocalizationPose3d)
                        .setLatencySupplier(this::supplyLocalizationLatency)
                        .setVisibleTargetsSupplier(this::supplyVisibleTargets)
                        .setAverageDistanceSupplier(this::supplyAverageDistance)
                        .setOrientationConsumer(this::setRobotOrientation)
                        .setUpdateHook(this::refreshLocalizationSnapshot)
                        .setRobotToCameraTransform(config.cameraRobotSpace())
                        .setDisplayHorizontalFov(DEFAULT_HORIZONTAL_FOV_DEG)
                        .setDisplayRangeMeters(Math.max(config.trustDistance(), 2.0))
                        .setConfidenceSupplier(config::confidence)
                        .setTargetYawSupplier(this::supplyTargetYaw)
                        .setTargetPitchSupplier(this::supplyTargetPitch)
                        .setTagDistanceSupplier(this::supplyPrimaryTagDistance)
                        .setTagIdSupplier(this::supplyPrimaryTagId)
                        .setRoles(config.roles())
                        .setFieldLayout(fieldLayout);

        VisionCamera camera = new VisionCamera(cameraConfig);
        camera.registerCapability(VisionCameraCapability.LIMELIGHT_CAMERA, this);
        camera.registerCapability(VisionCameraCapability.LOCALIZATION_SOURCE, this);
        camera.registerCapability(VisionCameraCapability.TARGETING_SOURCE, this);
        camera.registerCapability(VisionCameraCapability.FIDUCIAL_SOURCE, this);
        camera.registerCapability(VisionCameraCapability.LED, new LimelightLedCapability());
        camera.registerCapability(VisionCameraCapability.PIPELINE, new LimelightPipelineCapability());
        camera.registerCapability(VisionCameraCapability.STREAM, new LimelightStreamCapability());
        return camera;
    }

    private void refreshLocalizationSnapshot() {
        PoseEstimateWithLatency estimate = getPoseEstimate(config.localizationEstimator());
        if (estimate == null) {
            latestSnapshot = LocalizationSnapshot.empty();
            return;
        }
        Pose2d pose = estimate.getLocalizationPose();
        double latencySeconds = Timer.getFPGATimestamp() - (estimate.getLatency() / 1000.0);
        int tagCount = estimate.getTagCount();
        double distance = estimate.getDistToTag();
        latestSnapshot = new LocalizationSnapshot(pose, new Pose3d(pose), latencySeconds, tagCount, distance);
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
        if (!hasValidTarget()) {
            return Double.NaN;
        }
        double yaw = getCrosshairIndependentYaw();
        return Double.isFinite(yaw) ? yaw : getTargetHorizontalOffset();
    }

    private double supplyTargetPitch() {
        if (!hasValidTarget()) {
            return Double.NaN;
        }
        double pitch = getCrosshairIndependentPitch();
        return Double.isFinite(pitch) ? pitch : getTargetVerticalOffset();
    }

    private double supplyPrimaryTagDistance() {
        return latestSnapshot.visibleTargets() > 0
                ? latestSnapshot.averageDistanceMeters()
                : Double.NaN;
    }

    private int supplyPrimaryTagId() {
        return hasValidTarget() ? (int) getAprilTagID() : -1;
    }

    private double getCrosshairIndependentYaw() {
        double yaw = txnc != null ? txnc.getDouble(Double.NaN) : Double.NaN;
        if (Double.isFinite(yaw)) {
            return yaw;
        }
        return calculateYawFromCameraSpace(getTargetPoseCameraSpace());
    }

    private double getCrosshairIndependentPitch() {
        double pitch = tync != null ? tync.getDouble(Double.NaN) : Double.NaN;
        if (Double.isFinite(pitch)) {
            return pitch;
        }
        return calculatePitchFromCameraSpace(getTargetPoseCameraSpace());
    }

    private Double[] getTargetPoseCameraSpace() {
        PoseEstimate estimate = getPoseEstimate(PoseEstimateType.TARGET_POSE_CAMERA_SPACE);
        if (estimate == null) {
            return null;
        }
        Double[] raw = estimate.getRaw();
        if (raw == null || raw.length < 6) {
            return null;
        }
        return raw;
    }

    private double calculateYawFromCameraSpace(Double[] targetPose) {
        if (targetPose == null) {
            return Double.NaN;
        }
        double x = targetPose[0];
        double y = targetPose[1];
        if (!Double.isFinite(x) || !Double.isFinite(y)) {
            return Double.NaN;
        }
        return Math.toDegrees(Math.atan2(y, x));
    }

    private double calculatePitchFromCameraSpace(Double[] targetPose) {
        if (targetPose == null) {
            return Double.NaN;
        }
        double x = targetPose[0];
        double z = targetPose[2];
        if (!Double.isFinite(x) || !Double.isFinite(z)) {
            return Double.NaN;
        }
        return Math.toDegrees(Math.atan2(z, x));
    }

    private class LimelightLedCapability implements VisionCamera.LedControl {

        @Override
        public void setLedMode(VisionCamera.LedMode mode) {
            LimeLight.this.setLedMode(toVendorLedMode(mode));
        }

        @Override
        public VisionCamera.LedMode getLedMode() {
            return toGenericLedMode(LimeLight.this.getLedMode());
        }
    }

    private class LimelightPipelineCapability implements PipelineControl {
        @Override
        public void setPipeline(int pipeline) {
            LimeLight.this.setPipeline(pipeline);
        }

        @Override
        public int getPipeline() {
            return (int) LimeLight.this.getPipeline();
        }
    }

    private class LimelightStreamCapability implements StreamControl {
        @Override
        public void setStreamMode(VisionCamera.StreamMode mode) {
            LimeLight.this.setStream(toVendorStreamMode(mode));
        }

        @Override
        public VisionCamera.StreamMode getStreamMode() {
            return toGenericStreamMode(LimeLight.this.getStreamMode());
        }
    }

    private static VisionCamera.LedMode toGenericLedMode(LedMode mode) {
        return switch (mode) {
            case PIPELINE -> VisionCamera.LedMode.PIPELINE;
            case OFF -> VisionCamera.LedMode.OFF;
            case BLINK -> VisionCamera.LedMode.BLINK;
            case ON -> VisionCamera.LedMode.ON;
        };
    }

    private static LedMode toVendorLedMode(VisionCamera.LedMode mode) {
        return switch (mode) {
            case PIPELINE -> LedMode.PIPELINE;
            case OFF -> LedMode.OFF;
            case BLINK -> LedMode.BLINK;
            case ON -> LedMode.ON;
        };
    }

    private static VisionCamera.StreamMode toGenericStreamMode(StreamMode mode) {
        return switch (mode) {
            case STANDARD -> VisionCamera.StreamMode.STANDARD;
            case PIP_MAIN -> VisionCamera.StreamMode.PIP_MAIN;
            case PIP_SECONDARY -> VisionCamera.StreamMode.PIP_SECONDARY;
        };
    }

    private static StreamMode toVendorStreamMode(VisionCamera.StreamMode mode) {
        return switch (mode) {
            case STANDARD -> StreamMode.STANDARD;
            case PIP_MAIN -> StreamMode.PIP_MAIN;
            case PIP_SECONDARY -> StreamMode.PIP_SECONDARY;
        };
    }
    public boolean hasBotPose(){
        // return botpose.getNumberArray(null) != null;
        return false;
    }

    /**
     * Horizontal Offset From Crosshair To Target (LL1: -27 degrees to 27 degrees | LL2: -29.8 to 29.8 degrees)
     */
    public double getTargetHorizontalOffset(){
        return tx.getDouble(0);
    }

    /**
     * Vertical Offset From Crosshair To Target (LL1: -20.5 degrees to 20.5 degrees | LL2: -24.85 to 24.85 degrees)
     */
    public double getTargetVerticalOffset(){
        return ty.getDouble(0);
    }

     /**
     * Target Area (0% of image to 100% of image)
     */
    public double getTargetArea(){
        return ta.getDouble(0);
    }

    
     /**
     * Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getTargetSkew(){
        var data = t2d.getDoubleArray(new Double[]{});
        if(data.length == 0) return 0;
        return data[16];
    }

      /**
     * Skew or rotation (-90 degrees to 0 degrees)
     */
    public Double[] getT2D(){
        return t2d.getDoubleArray(new Double[]{});
    }

     /**
     * The pipeline’s latency contribution (ms) Add at least 11ms for image capture latency.
     */
    public double getLatency(){
        return tl.getDouble(0);
    }

     /**
     * Sidelength of shortest side of the fitted bounding box (pixels)
     */
    public double getTargetShortestSide(){
        return tshort.getDouble(0);
    }

     /**
     * Sidelength of longest side of the fitted bounding box (pixels)
     */
    public double getTargetLongestSide(){
        return tlong.getDouble(0);
    }

     /**
     * Horizontal sidelength of the rough bounding box (0 - 320 pixels)
     */
    public double getTargetHorizontalSide(){
        return thor.getDouble(0);
    }

     /**
     * Vertical sidelength of the rough bounding box (0 - 320 pixels)
     */
    public double getTargetVerticalSide(){
        return thor.getDouble(0);
    }

     /**
     * True active pipeline index of the camera (0 .. 9)
     */
    public double getPipeline(){
        return getpipe.getDouble(0);
    }


     /**
     * 2D Crosshairs [cx0, cy0, cx1, cy1]
     */
    public Double[] getCrosshairs(){
        return crosshairs.getDoubleArray(new Double[]{});
    }

     /**
     * 2D Crosshairs [cx0, cy0, cx1, cy1]
     */
    public void setCrosshairs(Double[] value){
         crosshairs.setDoubleArray(value);
    }


     /**
     * Camera transform in target space of primary apriltag or solvepnp target, NumberArray: Translation (x,y,z) Rotation(pitch,yaw,roll)
     */
    public Number[] getCameraTransform(){
        return camtran.getNumberArray(null);
    }

    /**
     * ID of primary AprilTag
     */
    public long getAprilTagID(){
        return tid.getInteger(0);
    }

     /**
     * Full JSON dump of targeting results
     */
    public String getJSON(){
        return json.getString("");
    }

     /**
     * Robot transform in field-space. Translation (X,Y,Z) Rotation(X,Y,Z), total latency, tag count, average tag distance from camera, average tag area, 
     */
    public PoseEstimate getPoseEstimate(PoseEstimateType type)
    {
        PoseEstimate estimate = new PoseEstimate(type);
        return estimate;
    }

    public PoseEstimateWithLatency getPoseEstimate(PoseEstimateWithLatencyType type)
    {
        PoseEstimateWithLatency estimate = new PoseEstimateWithLatency(type);
        return estimate;
    }
    
    public int getPriorityID(){
        return (int) priorityid.getInteger(-1);
    }    

    /**
     * Class ID of primary neural detector result or neural classifier result
     */
    public long getNeuralClassID(){
        return tid.getInteger(0);
    }

    /**
     *  Get the average HSV color underneath the crosshair region as a NumberArray
     */
    public Number[] getAverageHSV(){
        return tc.getNumberArray(null);
    }

    /**
     *  Sets limelight’s LED state
     */
    public void setLedMode(LedMode mode){
        ledMode.setNumber(mode.get());
    }

    public LedMode getLedMode() {
        int raw = (int) ledMode.getDouble(LedMode.PIPELINE.get());
        for (LedMode value : LedMode.values()) {
            if (value.get() == raw) {
                return value;
            }
        }
        return LedMode.PIPELINE;
    }

    /**
     *  Sets limelight’s operation mode
     */
    public void setCameraMode(CameraMode mode){
        camMode.setNumber(mode.get());
    }

    /**
     *  Sets limelight’s streaming mode
     */
    public void setStream(StreamMode mode){
        stream.setNumber(mode.get());
    }

    public StreamMode getStreamMode() {
        int raw = (int) stream.getDouble(StreamMode.STANDARD.get());
        for (StreamMode value : StreamMode.values()) {
            if (value.get() == raw) {
                return value;
            }
        }
        return StreamMode.STANDARD;
    }

     /**
     *  Sets limelight’s current pipeline (0-9)
     */
    public void setPipeline(int mode){
        pipeline.setNumber(mode);
    }

    public Number[] getRawFiducials(){
        return rawfiducials.getNumberArray(new Number[]{});
    }

    /**
     * Allows users to take snapshots during a match
     */
    public void setSnapshots(int mode){
        snapshot.setNumber(mode);
    }

    public double getDistanceFromTarget(double targetHeightMeters){
        return getDistanceFromTarget(config.cameraRobotSpace().getRotation().getY(), config.cameraRobotSpace().getTranslation().getZ(), targetHeightMeters);
    }

    public double getDistanceFromTarget(double mountingAngle, double mountingHeightMeters, double targetHeightMeters){
        double angleToTargetRadains =  (mountingAngle + getTargetVerticalOffset()) * (Math.PI/180d); 
        double heightDiff = targetHeightMeters - mountingHeightMeters;
        
        return heightDiff == 0 ? Math.tan(angleToTargetRadains) : (heightDiff)/Math.tan(angleToTargetRadains);
    }

    public void setPriorityID(int tag_id){
        priorityid.setInteger(tag_id);
    }

    public void setBotPoseTargetSpace(Double[] pose)
    {
        botpose_targetspace_set.setDoubleArray(pose);
    }
    public void setCameraPoseRobotSpace(Double[] pose)
    {
        camerapose_robotspace_set.setDoubleArray(pose);
    }

    private void applyCameraTransformToNetworkTables(Transform3d transform) {
        if (transform == null) return;
        Double[] pose = new Double[] {
            transform.getX(),
            transform.getY(),
            transform.getZ(),
            Math.toDegrees(transform.getRotation().getX()),
            Math.toDegrees(transform.getRotation().getY()),
            Math.toDegrees(transform.getRotation().getZ())
        };
        setCameraPoseRobotSpace(pose);
    }

    public void setRobotOrientation(Pose2d pose){
        robot_orientation_set.setDoubleArray(new Double[] {pose.getRotation().getDegrees(),0d,0d,0d,0d,0d});
    }

    @Override
    public double getLocalizationLatency() {
        return localizationCamera.getLocalizationLatency();
    }

    @Override
    public Pose2d getLocalizationPose() {
        return localizationCamera.getLocalizationPose();
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
        return tv.getDouble(0) == 1;
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

    public void setFiducialIdFilters(double[] array){
        fiducial_id_filters_set.setDoubleArray(array);
    }

    public Double[] getFiducialIdFilters(){
        return fiducial_id_filters_set.getDoubleArray(new Double[]{});
    }

    @Override
    public Matrix<N3, N1> getLocalizationStdDevs() {
        return localizationCamera.getLocalizationStdDevs();
    }
    
    public void setStdDevs(Matrix<N3, N1> single, Matrix<N3, N1> multi) {
        localizationCamera.setStdDevs(single, multi);
    }

    public Matrix<N3, N1> getSingleStdDev() {
        return localizationCamera.getSingleStdDev();
    }

    public Matrix<N3, N1> getMultiStdDev() {
        return localizationCamera.getMultiStdDev();
    }
    public VisionCamera getVisionCamera() {
        return localizationCamera;
    }

    public void setUseForLocalization(boolean useForLocalization) {
        this.useForLocalization = useForLocalization;
        localizationCamera.setUseForLocalization(useForLocalization);
    }

    public boolean isUseForLocalization() {
        return useForLocalization;
    }
}
