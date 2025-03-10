package ca.frc6390.athena.sensors.camera.limelight;

import ca.frc6390.athena.sensors.camera.LocalizationCamera;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight implements LocalizationCamera{
    public LimeLightConfig config;
    private NetworkTable limelightTable;
    public NetworkTableEntry tv;
    public NetworkTableEntry tx;
    public NetworkTableEntry ty;
    public NetworkTableEntry txnc;
    public NetworkTableEntry tync;
    public NetworkTableEntry botpose_targetspace_set;
    public NetworkTableEntry ta;
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

        public double getLatency()
        {
            return getRaw()[6];
        }
        public double getTagCount()
        {
            return getRaw()[7];
        }
        public double getDistToTag()
        {
            return getRaw()[8];
        }
        public double getAvgTagArea()
        {
            return getRaw()[9];
        }
        
    }

    public class PoseEstimate
    {
        Double[] dub = new Double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
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
            Translation2d translation = new Translation2d(poseReal[2], poseReal[0]);
            Rotation2d rotation2d = Rotation2d.fromDegrees(poseReal[5]);
            return new Pose2d(translation, rotation2d);
        }

        public Pose2d getLocalizationPose()
        {
            Double[] poseReal = getRaw();
            if (poseReal == null) return new Pose2d();
            Translation2d translation = new Translation2d(poseReal[0], poseReal[1]);
            Rotation2d rotation2d = Rotation2d.fromDegrees(poseReal[5]);
            return new Pose2d(translation, rotation2d);
        }
    }

    public LimeLight(LimeLightConfig config){
        this.config = config;
        limelightTable = NetworkTableInstance.getDefault().getTable(config.table());
        tv = limelightTable.getEntry("tv");
        tx = limelightTable.getEntry("tx");
        ty = limelightTable.getEntry("ty");
        ta = limelightTable.getEntry("ta");
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

        setFiducialIdFilters(config.ignoreTags());
    }
    
    
    /**
     * Whether the limelight has any valid targets
     */
    public boolean hasValidTarget(){
        return tv.getDouble(0) == 1;
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
        return t2d.getDoubleArray(new Double[]{})[16];
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

    public void setRobotOrientation(Double[] orientation){
        robot_orientation_set.setDoubleArray(orientation);
    }

    @Override
    public double getLocalizationLatency() {
        return getPoseEstimate(config.localizationEstimator()).getLatency(); 
    }

    @Override
    public Pose2d getLocalizationPose() {
        return getPoseEstimate(config.localizationEstimator()).getLocalizationPose(); 
    }

    public void setFiducialIdFilters(double[] array){
        fiducial_id_filters_set.setDoubleArray(array);
    }

    public Double[] getFiducialIdFilters(){
        return fiducial_id_filters_set.getDoubleArray(new Double[]{});
    }
}