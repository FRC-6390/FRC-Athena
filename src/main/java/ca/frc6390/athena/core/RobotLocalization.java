package ca.frc6390.athena.core;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import choreo.auto.AutoFactory;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
public class RobotLocalization extends SubsystemBase implements RobotSendableSystem{
    
    public record RobotLocalizationConfig(double xStd, double yStd, double thetaStd, double vXStd, double vYStda, double vThetaStd, double v2XStd, double v2YStda, double v2ThetaStd, PIDConstants translation, PIDConstants rotation, boolean useVision ,double slipThresh) {

        public RobotLocalizationConfig(double xStd, double yStd, double thetaStd, double vXStd, double vYStda, double vThetaStd) {
            this(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, vXStd, vYStda, vThetaStd, new PIDConstants(0), new PIDConstants(0), true,0.2);
        }

        public RobotLocalizationConfig(double xStd, double yStd, double thetaStd) {
            this(xStd, yStd, thetaStd, 0.9, 0.9, 0.9);
        }

        public RobotLocalizationConfig() {
            this(0.1,0.1,0.001);
        }

        public RobotLocalizationConfig setAutoPlannerPID(PIDConstants translation, PIDConstants rotation){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, v2XStd, v2YStda, v2ThetaStd, translation, rotation, useVision,slipThresh);
        }

        public RobotLocalizationConfig setVision(double vXStd, double vYStda, double vThetaStd){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, v2XStd, v2YStda, v2ThetaStd, translation, rotation, useVision,slipThresh);
        }

        public RobotLocalizationConfig setVisionMultitag(double v2XStd, double v2YStda, double v2ThetaStd){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, v2XStd, v2YStda, v2ThetaStd, translation, rotation, useVision,slipThresh);
        }

        public RobotLocalizationConfig setVisionEnabled(boolean useVision){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, v2XStd, v2YStda, v2ThetaStd, translation, rotation, useVision,slipThresh);
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

        public RobotLocalizationConfig setSlipThresh(double slipThresh){
            return new RobotLocalizationConfig(xStd, yStd, thetaStd, vXStd, vYStda, vThetaStd, v2XStd, v2YStda, v2ThetaStd, translation, rotation, useVision,slipThresh);
        }
    }

    private final SwerveDrivePoseEstimator fieldEstimator, relativeEstimator;
    private final SwerveDrivetrain drivetrain;
    private RobotVision vision;

    private Pose2d fieldPose, relativePose;
    private Field2d field;
    private RobotConfig robotConfig;

    private RobotLocalizationConfig localizationConfig;
    private boolean visionEnabled = false;
    private PIDController rotationController, translationController;
    private AutoFactory factory;


    public RobotLocalization(SwerveDrivetrain drivetrain, RobotVision vision, RobotLocalizationConfig config, Pose2d pose) {
        this.localizationConfig = config;
        this.fieldPose = pose;
        this.relativePose = pose;
      
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.field = new Field2d();

        this.visionEnabled = config.useVision;
        config = config == null ? new RobotLocalizationConfig() : config;

        fieldEstimator = new SwerveDrivePoseEstimator(drivetrain.getKinematics(), drivetrain.getIMU().getYaw(), drivetrain.getSwerveModulePositions(), pose, config.getStd(), config.getVisionStd());
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
            robotConfig = RobotConfig.fromGUISettings();  }catch(Exception e){
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

    public RobotLocalization configureChoreo(PIDConstants translationConstants, PIDConstants rotationConstants){
        rotationController = new PIDController(rotationConstants.kP, rotationConstants.kI, rotationConstants.kD);
        rotationController.setIZone(rotationConstants.iZone);

        translationController = new PIDController(translationConstants.kP, translationConstants.kI, translationConstants.kD);
        translationController.setIZone(translationConstants.iZone);

        factory = new AutoFactory(this::getFieldPose, this::resetFieldPose, 
        (sample) -> {
            Pose2d botpose = getFieldPose();
            Pose2d trajpose = sample.getPose();
            
            ChassisSpeeds speeds = sample.getChassisSpeeds();

            speeds.vxMetersPerSecond = translationController.calculate(botpose.getX(), trajpose.getX());
            speeds.vyMetersPerSecond = translationController.calculate(botpose.getY(), trajpose.getY());
            speeds.omegaRadiansPerSecond = rotationController.calculate(botpose.getRotation().getRadians(), trajpose.getRotation().getRadians());

            drivetrain.getRobotSpeeds().setAutoSpeeds(speeds);
        }, 
        true, 
        drivetrain);
      return this;
    }

    public AutoFactory getChoreoAutoFactory(){
        return factory;
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
            vision.setRobotOrientation(drivetrain.getIMU().getVirtualAxis("field"));
            
            List<Pose2d> poses = vision.getLocalizationPoses();

            SmartDashboard.putNumber("Localization Poses", poses.size());
            if(poses.size() < 2){
                fieldEstimator.setVisionMeasurementStdDevs(localizationConfig.getVisionMultitagStd());
            }else{
                fieldEstimator.setVisionMeasurementStdDevs(localizationConfig.getVisionStd());
            }

            vision.setLocalizationPoses(fieldEstimator::addVisionMeasurement);
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

    public Field2d getField() {
        return field;
    }

    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
        tab.add("Estimator", field).withPosition( 0,0).withSize(4, 3);
        tab.addDouble("Field X", () -> getFieldPose().getX()).withPosition( 4,2);
        tab.addDouble("Field Y", () -> getFieldPose().getY()).withPosition( 5,2);
        tab.addDouble("Field Theta", () -> getFieldPose().getRotation().getDegrees()).withPosition( 4,0).withSize(2, 2);
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
