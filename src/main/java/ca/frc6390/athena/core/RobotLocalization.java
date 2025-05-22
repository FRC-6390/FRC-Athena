package ca.frc6390.athena.core;

import java.util.function.BiConsumer;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;

import ca.frc6390.athena.devices.IMU;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
public class RobotLocalization<T> extends SubsystemBase implements RobotSendableSystem{
    
    public record RobotLocalizationConfig(double xStd, double yStd, double thetaStd, double vXStd, double vYStda, double vThetaStd, double v2XStd, double v2YStda, double v2ThetaStd, PIDConstants translation, PIDConstants rotation, boolean useVision) {

        public RobotLocalizationConfig(double xStd, double yStd, double thetaStd, double vXStd, double vYStda, double vThetaStd) {
            this(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, vXStd, vYStda, vThetaStd, new PIDConstants(0), new PIDConstants(0), true);
        }

        public RobotLocalizationConfig(double xStd, double yStd, double thetaStd) {
            this(xStd, yStd, thetaStd, 0.9, 0.9, 0.9);
        }

        public RobotLocalizationConfig() {
            this(0.1,0.1,0.001);
        }

        public static RobotLocalizationConfig vision(double vXStd, double vYStda, double vThetaStd){
            return new RobotLocalizationConfig().setVision(vXStd, vYStda, vThetaStd).setVisionEnabled(true);
        }

        public static RobotLocalizationConfig defualt(){
            return new RobotLocalizationConfig();
        }

        public RobotLocalizationConfig setAutoPlannerPID(double tP, double tI, double tD, double rP, double rI, double rD){
            return setAutoPlannerPID(new PIDConstants(tP, tI, tD), new PIDConstants(rP, rI, rD));
        }

        public RobotLocalizationConfig setAutoPlannerPID(PIDConstants translation, PIDConstants rotation){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, v2XStd, v2YStda, v2ThetaStd, translation, rotation, useVision);
        }

        public RobotLocalizationConfig setVision(double vXStd, double vYStda, double vThetaStd){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, v2XStd, v2YStda, v2ThetaStd, translation, rotation, useVision);
        }

        public RobotLocalizationConfig setVisionMultitag(double v2XStd, double v2YStda, double v2ThetaStd){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, v2XStd, v2YStda, v2ThetaStd, translation, rotation, useVision);
        }

        public RobotLocalizationConfig setVisionEnabled(boolean useVision){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, v2XStd, v2YStda, v2ThetaStd, translation, rotation, useVision);
        }

        public Matrix<N3, N1> getStd(){
            return VecBuilder.fill(xStd,yStd,Units.degreesToRadians(thetaStd));
        }

        public Matrix<N3, N1> getVisionStd(){
            return VecBuilder.fill(vXStd,vYStda,Units.degreesToRadians(vThetaStd));
        }
        public Matrix<N3, N1> getVisionMultitagStd(){
            return VecBuilder.fill(v2XStd,v2YStda,Units.degreesToRadians(v2ThetaStd));
        }
    }

    private final PoseEstimator<T> fieldEstimator, relativeEstimator;
    private RobotVision vision;
    private final IMU imu;
    private final RobotSpeeds robotSpeeds;
    private final Supplier<T> wheelPositions;

    private Pose2d fieldPose, relativePose;
    private Field2d field;
    private RobotConfig robotConfig;

    private RobotLocalizationConfig localizationConfig;
    private boolean visionEnabled = false;
    private PIDController rotationController, translationController;
    private AutoFactory factory;

    private BiConsumer<ChassisSpeeds, DriveFeedforwards> autoDrive;

    private boolean suppressUpdates = false;

    public RobotLocalization(PoseEstimator<T> fieldEstimator, PoseEstimator<T> relativeEstimator, RobotLocalizationConfig config, RobotSpeeds robotSpeeds, IMU imu, Supplier<T> wheelPositions) {
        this.localizationConfig = config;
        this.robotSpeeds = robotSpeeds;
        this.imu = imu;
        this.wheelPositions = wheelPositions;
        this.fieldPose = new Pose2d();
        this.relativePose = new Pose2d();
        this.autoDrive = (speeds, feed) ->  robotSpeeds.setSpeeds("auto",speeds);
      
        this.field = new Field2d();

        this.visionEnabled = config.useVision;
        config = config == null ? new RobotLocalizationConfig() : config;

        
        this.fieldEstimator = fieldEstimator;
        this.relativeEstimator = relativeEstimator;
 
        imu.addVirtualAxis("relative", imu::getYaw);
        imu.addVirtualAxis("field", imu::getYaw);
        imu.setVirtualAxis("relative", new Rotation2d());
        imu.setVirtualAxis("field", new Rotation2d());

        if(config.rotation != null && config.translation != null){
            configurePathPlanner(config.translation, config.rotation);
        }  
    }

    public RobotLocalization<T> enableVisionForLocalization(boolean visionEnabled){
        this.visionEnabled = visionEnabled;
        return this;
    }

    public RobotLocalization<T> setRobotVision(RobotVision vision){
        this.vision = vision;
        vision.setLocalizationStdDevs(localizationConfig.getVisionStd(), localizationConfig.getVisionMultitagStd());
        return this;
    }

    public void setAutoDrive(BiConsumer<RobotSpeeds, ChassisSpeeds> autoDrive) {
        this.autoDrive = (speeds, feeds) -> autoDrive.accept(robotSpeeds, speeds);
    }

    public RobotLocalization<T> configurePathPlanner(PIDConstants translationConstants, PIDConstants rotationConstants){
      return configurePathPlanner(translationConstants, rotationConstants, autoDrive);
    }

    public RobotLocalization<T> configurePathPlanner(PIDConstants translationConstants, PIDConstants rotationConstants, BiConsumer<ChassisSpeeds, DriveFeedforwards> output){
        try{
            robotConfig = RobotConfig.fromGUISettings();  
        }catch(Exception e){
            DriverStation.reportWarning("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
            return this;
        }
        AutoBuilder.configure(
            this::getFieldPose, 
            this::resetFieldPose, 
            () -> robotSpeeds.getSpeeds("drive"), 
            output, 
            new PPHolonomicDriveController(
            translationConstants,
            rotationConstants
            ),
            robotConfig,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            }            
      );
      FollowPathCommand.warmupCommand().schedule();
      return this;
    }

    public RobotLocalization<T> configureChoreo(Subsystem drivetrain){
        rotationController = new PIDController(localizationConfig.rotation.kP, localizationConfig.rotation.kI, localizationConfig.rotation.kD);
        rotationController.setIZone(localizationConfig.rotation.iZone);

        translationController = new PIDController(localizationConfig.translation.kP, localizationConfig.translation.kI, localizationConfig.translation.kD);
        translationController.setIZone(localizationConfig.translation.iZone);

        factory = new AutoFactory(this::getFieldPose, this::resetFieldPose, 
        (sample) -> {
            Pose2d botpose = getFieldPose();
            Pose2d trajpose = sample.getPose();
            
            ChassisSpeeds speeds = sample.getChassisSpeeds();

            speeds.vxMetersPerSecond = translationController.calculate(botpose.getX(), trajpose.getX());
            speeds.vyMetersPerSecond = translationController.calculate(botpose.getY(), trajpose.getY());
            speeds.omegaRadiansPerSecond = rotationController.calculate(botpose.getRotation().getRadians(), trajpose.getRotation().getRadians());

            robotSpeeds.setSpeeds("auto", speeds);
        }, 
        true,
        drivetrain);
      return this;
    }

    public AutoFactory getChoreoAutoFactory(){
        return factory;
    }

    public void resetFieldPose(Pose2d pose) {
        fieldEstimator.resetPosition(pose.getRotation(), wheelPositions.get(), pose);
        imu.setVirtualAxis("field", pose.getRotation());
        this.fieldPose = pose;
    }

    public void resetFieldPose(double x, double y) {
        resetFieldPose(new Pose2d(x, y, imu.getVirtualAxis("field")));
    }

    public void resetFieldPose(double x, double y, double thetaDegree) {
        resetFieldPose(new Pose2d(x, y, Rotation2d.fromDegrees(thetaDegree)));
    }

    public void resetFieldPose() {
        resetFieldPose(new Pose2d(0,0, new Rotation2d(0)));
    }

    public void resetRelativePose(Pose2d pose) {
        relativeEstimator.resetPosition(pose.getRotation(), wheelPositions.get(), pose);
        imu.setVirtualAxis("relative", pose.getRotation());
        this.relativePose = pose;
    }

    public void resetRelativePose(double x, double y) {
        resetRelativePose(new Pose2d(x, y, imu.getVirtualAxis("relative")));
    }

    public void resetRelativePose(double x, double y, double r) {
        resetRelativePose(new Pose2d(x, y, new Rotation2d(r)));
    }

    public void resetRelativePose() {
        resetRelativePose(0,0, 0);
    }

    public void setVisionStd(Matrix<N3, N1> matrix){
        fieldEstimator.setVisionMeasurementStdDevs(matrix);
    }

    public void setVisionStd(double x, double y, double theta){
        setVisionStd(VecBuilder.fill(x,y,Units.degreesToRadians(theta)));
    }

    public void update() {
        
        if(suppressUpdates) return;

        if(vision != null && visionEnabled) {
            vision.setRobotOrientation(fieldPose);
            // vision.getLimelights().forEach((table, ll) -> ll.setFiducialIdFilters(Arrays.stream(ll.config.filteredTags()).mapToDouble(i -> i).toArray()));
            // List<Pose2d> poses = vision.getLocalizationPoses();
            // SmartDashboard.putNumber("Localization Poses", poses.size());
            vision.addLocalizationPoses(data -> {
                // System.out.println(data);
                if(data.pose() != null){
                    if (data.stdDevs() != null){
                        fieldEstimator.addVisionMeasurement(data.pose(), data.latency(), data.stdDevs());
                    }else {
                        fieldEstimator.addVisionMeasurement(data.pose(), data.latency());
                    }
                }
            });
        }

        fieldPose = fieldEstimator.update(imu.getVirtualAxis("field"), wheelPositions.get());
        relativePose = relativeEstimator.update(imu.getVirtualAxis("relative"), wheelPositions.get());
        field.setRobotPose(fieldPose);
    }

    public FieldObject2d getField2dObject(String id){
        return field.getObject(id);
    }

    public Pose2d getFieldPose(){
        return fieldPose;
    }

    public boolean isSuppressUpdates() {
        return suppressUpdates;
    }

    public void setSuppressUpdates(boolean suppressUpdates) {
        this.suppressUpdates = suppressUpdates;
    }

    public Pose2d getRelativePose(){
        return relativePose;
    }

    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level) {
        tab.add("Estimator", field).withPosition( 0,0).withSize(4, 3);

        if(level.equals(SendableLevel.DEBUG)) {
            tab.addDouble("Field X", () -> getFieldPose().getX()).withPosition( 4,2);
            tab.addDouble("Field Y", () -> getFieldPose().getY()).withPosition( 5,2);
            tab.addDouble("Field Theta", () -> getFieldPose().getRotation().getDegrees()).withPosition( 4,0).withSize(2, 2);
        }
        tab.addDouble("Relative X", () -> getRelativePose().getX()).withPosition( 6,2);
        tab.addDouble("Relative Y", () -> getRelativePose().getY()).withPosition( 7,2);
        tab.addDouble("Relative Theta", () -> getRelativePose().getRotation().getDegrees()).withPosition( 6,0).withSize(2, 2);
        return tab;
    }

    @Override
    public void periodic() {
       update();
    }

} 
