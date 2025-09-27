package ca.frc6390.athena.core;

import java.util.Arrays;
import java.util.HashMap;

import ca.frc6390.athena.commands.movement.RotateToAngle;
import ca.frc6390.athena.commands.movement.RotateToPoint;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.core.RobotLocalization.RobotLocalizationConfig;
import ca.frc6390.athena.core.RobotVision.RobotVisionConfig;
import ca.frc6390.athena.devices.IMU;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrain;
import ca.frc6390.athena.drivetrains.differential.DifferentialDrivetrainConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrainConfig;
import ca.frc6390.athena.mechanisms.Mechanism;
import ca.frc6390.athena.sensors.camera.ConfigurableCamera;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RobotBase<T extends RobotDrivetrain<T>> { //extends TimedRobot {
    
    public record RobotBaseConfig<T extends RobotDrivetrain<T>>(RobotDrivetrainConfig<T> driveTrain, RobotLocalizationConfig localizationConfig, RobotVisionConfig visionConfig) {
        
        public static RobotBaseConfig<SwerveDrivetrain> swerve(SwerveDrivetrainConfig config){
            return new RobotBaseConfig<>(config, null, null);
        }
        
        public static RobotBaseConfig<DifferentialDrivetrain> differential(DifferentialDrivetrainConfig config){
            return new RobotBaseConfig<>(config, null, null);
        }

        public RobotBaseConfig<T> setLocalization(RobotLocalizationConfig localizationConfig){
            return new RobotBaseConfig<>(driveTrain, localizationConfig, visionConfig);
        }

        public RobotBaseConfig<T> setVision(RobotVisionConfig visionConfig){
            return new RobotBaseConfig<>(driveTrain, localizationConfig, visionConfig);
        }

        public RobotBaseConfig<T> setVision(ConfigurableCamera... cameras){
            return new RobotBaseConfig<>(driveTrain, localizationConfig, RobotVisionConfig.defualt().addCameras(cameras));
        }

        public RobotBase<T> create(){
            return new RobotBase<>(this);
        }
    }

    private final RobotDrivetrain<T> drivetrain;
    private final RobotLocalization<?> localization;
    private final RobotVision vision;
    private final RobotAuto autos;
    private final HashMap<String, Mechanism> mechanisms;
    

    public RobotBase(RobotBaseConfig<T> config){
        drivetrain = config.driveTrain.build();
        localization = config.localizationConfig != null ? drivetrain.localization(config.localizationConfig()) : null;
        vision = config.visionConfig != null ? RobotVision.fromConfig(config.visionConfig) : null; 
        autos = new RobotAuto();
        mechanisms = new HashMap<>();

        if(localization != null && vision != null){
            localization.setRobotVision(vision);
            localization.configureChoreo(drivetrain);
        }
        
    }

    public RobotBase<T> registerMechanism(Mechanism... mechs){
        Arrays.stream(mechs).forEach(mech ->  mechanisms.put(mech.getName(), mech));
        return this;
    }

    public RobotLocalization<?> getLocalization(){
        return localization;
    }

    public T getDrivetrain(){
        return drivetrain.get();
    }

    public RobotVision getVision() {
        return vision;
    }

    public RobotBase<T> shuffleboard() {
        return shuffleboard("Drivetrain");
    }

    public RobotBase<T> shuffleboard(String drive) {
        return shuffleboard(drive, "Localization");
    }

    public RobotBase<T> shuffleboard(String drive, String local) {
        drivetrain.shuffleboard(drive);

        if(localization != null){
            localization.shuffleboard(local);
        }

        return this;
    }


    public LimeLight getCameraFacing(double x, double y){
        return getCameraFacing(new Translation2d(x, y));
    }

    public LimeLight getCameraFacing(Translation2d translation2d){

        if (vision == null || localization == null) {
            return null;
        }
    
        Pose2d pose = localization.getFieldPose();
        Rotation2d targetAngle = Rotation2d.fromRadians(Math.atan2(translation2d.getY() - pose.getY(), translation2d.getX() - pose.getX()));
        
        Rotation2d desiredRelativeAngle = targetAngle.minus(pose.getRotation());
    
        LimeLight bestCamera = null;
        double smallestAngleDiff = Double.MAX_VALUE;

        for (LimeLight camera : vision.getLimelights().values()) {
            Rotation2d cameraRelativeAngleRad = camera.config.getYawRelativeToForwards();
            double angleDiff = Math.abs(desiredRelativeAngle.minus(cameraRelativeAngleRad).getDegrees());
            if (angleDiff < smallestAngleDiff) {
                smallestAngleDiff = angleDiff;
                bestCamera = camera;
            }
        }
        return bestCamera;
    }

    public RotateToPoint rotateTo(double x, double y){
        return new RotateToPoint(this, x, y);
    }

    public RotateToAngle rotateTo(double degrees){
        return new RotateToAngle(this, degrees);
    }

    public RotateToAngle rotateBy(double degrees){
        return new RotateToAngle(this, getDrivetrain().getIMU().getYaw().plus(Rotation2d.fromDegrees(degrees)));
    }

    public RobotSpeeds getRobotSpeeds(){
        return drivetrain.getRobotSpeeds();
    }

    public IMU getIMU(){
        return drivetrain.getIMU();
    }

    public RobotBase<T> registerPathPlannerAuto(String... auto){
        autos.registerPathplannerAuto(auto);
        return this;
    }

    public SendableChooser<Command> registerAutoChooser(String defualtAuto){
        SmartDashboard.putData("Auto Chooser", autos.getSendableChooser(defualtAuto));
        return autos.getAutoChooser();
    }

    public RobotAuto getAutos(){
        return autos;
    }

    public void registerPIDCycles(TimedRobot robot){
        for (Mechanism mech : mechanisms.values()) {
            if (mech.isCustomPIDCycle()) robot.addPeriodic(mech::calculatePID, mech.getPidPeriod());
        }
    }

    public void resetPIDs(){
        for (Mechanism mech : mechanisms.values()) {
            mech.resetPID();
        }
    }

    public ChassisSpeeds createRobotRelativeSpeeds(double xSpeed, double ySpeed, double rot){
        return ChassisSpeeds.fromRobotRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rot), getLocalization().getRelativePose().getRotation());
    }

    public ChassisSpeeds createFieldRelativeSpeeds(double xSpeed, double ySpeed, double rot){
        return ChassisSpeeds.fromFieldRelativeSpeeds(new ChassisSpeeds(xSpeed, ySpeed, rot), getLocalization().getRelativePose().getRotation());
    }
}
