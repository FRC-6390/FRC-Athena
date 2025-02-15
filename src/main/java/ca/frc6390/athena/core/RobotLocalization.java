package ca.frc6390.athena.core;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatency;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight.PoseEstimateWithLatencyType;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
public class RobotLocalization extends SubsystemBase implements RobotSendableSystem{
    
    public record RobotLocalizationConfig(double xStd, double yStd, double thetaStd, double vXStd, double vYStda, double vThetaStd, PoseEstimateWithLatencyType pose, PIDConstants translation, PIDConstants rotation, boolean useVision) {

        public RobotLocalizationConfig(double xStd, double yStd, double thetaStd, double vXStd, double vYStda, double vThetaStd) {
            this(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, PoseEstimateWithLatencyType.BOT_POSE_BLUE, new PIDConstants(0), new PIDConstants(0), true);
        }

        public RobotLocalizationConfig(double xStd, double yStd, double thetaStd) {
            this(xStd, yStd, thetaStd, 0.9, 0.9, 0.9);
        }

        public RobotLocalizationConfig() {
            this(0.1,0.1,0.001);
        }

        public RobotLocalizationConfig setPathPlannerPID(PIDConstants translation, PIDConstants rotation){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, pose, translation, rotation, useVision);
        }

        public RobotLocalizationConfig setPoseEstimateOrigin(PoseEstimateWithLatencyType pose){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, pose, translation, rotation, useVision);
        }

        public RobotLocalizationConfig setVision(double vXStd, double vYStda, double vThetaStd){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, pose, translation, rotation, useVision);
        }

        public RobotLocalizationConfig setVisionEnabled(boolean useVision){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, pose, translation, rotation, useVision);
        }

        public Matrix<N3, N1> getStd(){
            return VecBuilder.fill(xStd,yStd,Units.degreesToRadians(thetaStd));
        }

        public Matrix<N3, N1> getVisionStd(){
            return VecBuilder.fill(vXStd,vYStda,Units.degreesToRadians(vThetaStd));
        }
    }

    private final SwerveDrivePoseEstimator fieldEstimator, relativeEstimator;
    private final SwerveDrivetrain drivetrain;
    private RobotVision vision;
    private final PoseEstimateWithLatencyType estimateWithLatencyType;
    private Pose2d fieldPose, relativePose;
    private Field2d field;
    private RobotConfig config;
    private boolean visionEnabled = false;

    public RobotLocalization(SwerveDrivetrain drivetrain, RobotVision vision, RobotLocalizationConfig config, Pose2d pose) {
        this.fieldPose = pose;
        this.relativePose = pose;
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.field = new Field2d();
        estimateWithLatencyType = config.pose;
        this.visionEnabled = config.useVision;

        fieldEstimator = config != null ?
         new SwerveDrivePoseEstimator(drivetrain.getKinematics(), drivetrain.getIMU().getYaw(), drivetrain.getSwerveModulePositions(), pose, config.getStd(), config.getVisionStd()) : 
         new SwerveDrivePoseEstimator(drivetrain.getKinematics(), drivetrain.getIMU().getYaw(), drivetrain.getSwerveModulePositions(), pose);

        relativeEstimator = new SwerveDrivePoseEstimator(drivetrain.getKinematics(), drivetrain.getIMU().getYaw(), drivetrain.getSwerveModulePositions(), pose);

        drivetrain.getIMU().addVirtualAxis("relative", drivetrain.getIMU()::getYaw);
        drivetrain.getIMU().addVirtualAxis("field", drivetrain.getIMU()::getYaw);


        drivetrain.getIMU().setVirtualAxis("relative", new Rotation2d());
        drivetrain.getIMU().setVirtualAxis("field", new Rotation2d());

        if(config.rotation != null && config.translation != null){
            configurePathPlanner(config.translation, config.rotation);
        }
    }

    public RobotLocalization(SwerveDrivetrain drivetrain, RobotVision vision, RobotLocalizationConfig config) {
        this(drivetrain, vision, config, new Pose2d());
    }

    public RobotLocalization(SwerveDrivetrain drivetrain, RobotLocalizationConfig config) {
        this(drivetrain, null, config);
    }

    public RobotLocalization(SwerveDrivetrain drivetrain) {
        this(drivetrain, null, null);
    }

    public RobotLocalization enableVisionForLocalization(boolean visionEnabled){
        this.visionEnabled = visionEnabled;
        return this;
    }

    public RobotLocalization setRobotVision(RobotVision vision){
        this.vision = vision;
        return this;
    }

    public RobotLocalization configurePathPlanner(PIDConstants translationConstants, PIDConstants rotationConstants){
        try{
        config = RobotConfig.fromGUISettings();  }catch(Exception e){
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", e.getStackTrace());
        }
        AutoBuilder.configure(
            this::getFieldPose, 
            this::resetFieldPose, 
            () -> drivetrain.getRobotSpeeds().getDriverSpeeds(), 
            (speeds, feedforwards) -> drivetrain.getRobotSpeeds().setAutoSpeeds(speeds), 
            new PPHolonomicDriveController(
            translationConstants,
            rotationConstants
            ),
            config,
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

    public RobotLocalization configureChoreo(PIDConstants translationConstants, PIDConstants rotationConstants){
      return this;
    }

    public Pose2d getPose(){
        return new Pose2d(fieldPose.getX(), fieldPose.getY(), drivetrain.getIMU().getYaw());
    }

    public void resetFieldPose(Pose2d pose, Rotation2d heading) {
        drivetrain.getIMU().setVirtualAxis("field", heading);
        fieldEstimator.resetPosition(heading.unaryMinus(), drivetrain.getSwerveModulePositions(), pose);
        this.fieldPose = pose;
    }

    public void resetFieldPose(Pose2d pose) {
        resetFieldPose(pose, pose.getRotation());
    }

    public void resetFieldPose(double x, double y) {
        resetFieldPose(new Pose2d(x, y, drivetrain.getIMU().getVirtualAxis("field")));
    }

    public void resetFieldPose(double x, double y, double thetaDegree) {
        resetFieldPose(new Pose2d(x, y, Rotation2d.fromDegrees(thetaDegree)));
    }

    public void resetFieldPose() {
        resetFieldPose(new Pose2d(0,0, new Rotation2d(0)));
    }


    public void resetRelativePose(Pose2d pose, Rotation2d heading) {
        drivetrain.getIMU().setVirtualAxis("relative", heading);
        relativeEstimator.resetPosition(heading.unaryMinus(), drivetrain.getSwerveModulePositions(), pose);
        this.relativePose = pose;
    }

    public void resetRelativePose(Pose2d pose) {
        resetRelativePose(pose, pose.getRotation());
    }

    public void resetRelativePose(double x, double y) {
        resetRelativePose(new Pose2d(x, y, drivetrain.getIMU().getVirtualAxis("relative")));
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
        if(vision != null && visionEnabled) {
            Double[] orientation = {fieldPose.getRotation().getDegrees(),0d,0d,0d,0d,0d};
            for (String table : vision.getCameraTables()) {
                LimeLight camera = vision.getCamera(table);
                camera.setRobotOrientation(orientation);
                if(camera.hasValidTarget()){
                    PoseEstimateWithLatency data = camera.getPoseEstimate(estimateWithLatencyType);
                    double timestamp = Timer.getFPGATimestamp() - (data.getLatency() / 1000.0);
                    fieldEstimator.addVisionMeasurement(data.getPose(), timestamp);
                }
            }
        }

        fieldPose = fieldEstimator.update(drivetrain.getIMU().getVirtualAxis("field"), drivetrain.getSwerveModulePositions());
        relativePose = relativeEstimator.update(drivetrain.getIMU().getVirtualAxis("relative"), drivetrain.getSwerveModulePositions());
        field.setRobotPose(fieldPose);
    }

    public Pose2d getFieldPose(){
        return fieldPose;
    }

    public Pose2d getRelativePose(){
        return relativePose;
    }

    public Field2d getField2d() {
        return field;
    }

    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
        tab.add("Estimator", field).withPosition( 0,0).withSize(4, 3);
        tab.addDouble("Field X", () -> getFieldPose().getX()).withPosition( 4,2);
        tab.addDouble("Field Y", () -> getFieldPose().getY()).withPosition( 5,2);
        tab.addDouble("Field Theta", () -> getFieldPose().getRotation().getDegrees()).withWidget(BuiltInWidgets.kGyro).withPosition( 4,0).withSize(2, 2);
        tab.addDouble("Relative X", () -> getRelativePose().getX()).withPosition( 6,2);
        tab.addDouble("Relative Y", () -> getRelativePose().getY()).withPosition( 7,2);
        tab.addDouble("Relative Theta", () -> getRelativePose().getRotation().getDegrees()).withWidget(BuiltInWidgets.kGyro).withPosition( 6,0).withSize(2, 2);
        return tab;
    }

    @Override
    public void periodic() {
       update();
    }
} //IM A SCATMAN
