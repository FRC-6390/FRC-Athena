package ca.frc6390.athena.core;

import ca.frc6390.athena.commands.movement.RotateToAngle;
import ca.frc6390.athena.commands.movement.RotateToPoint;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.core.RobotLocalization.RobotLocalizationConfig;
import ca.frc6390.athena.core.RobotVision.RobotVisionConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain.SwerveDrivetrainConfig;
import ca.frc6390.athena.sensors.camera.limelight.LimeLight;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public class RobotBase<T extends RobotDrivetrain<T>> {
    
    public record RobotBaseConfig<T extends RobotDrivetrain<T>>(RobotDrivetrainConfig<T> driveTrain, RobotLocalizationConfig localizationConfig, RobotVisionConfig visionConfig) {
        
        public static RobotBaseConfig<SwerveDrivetrain> swerve(SwerveDrivetrainConfig config){
            return new RobotBaseConfig<>(config, null, null);
        }
        
        public RobotBaseConfig<T> setLocalization(RobotLocalizationConfig localizationConfig){
            return new RobotBaseConfig<>(driveTrain, localizationConfig, visionConfig);
        }

        public RobotBaseConfig<T> setVision(RobotVisionConfig visionConfig){
            return new RobotBaseConfig<>(driveTrain, localizationConfig, visionConfig);
        }

        public RobotBase<T> create(){
            return new RobotBase<>(this);
        }
    }

    private final RobotDrivetrain<T> drivetrain;
    private final RobotLocalization localization;
    private final RobotVision vision;

    public RobotBase(RobotBaseConfig<T> config){
        drivetrain = config.driveTrain.create();
        localization = config.localizationConfig != null ? drivetrain.localization(config.localizationConfig()) : null;
        vision = config.visionConfig != null ? RobotVision.fromConfig(config.visionConfig) : null; 

        if(localization != null && vision != null){
            localization.setRobotVision(vision);
        }
    }

    public RobotLocalization getLocalization(){
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
        return getCameraFacing(x, y, false);
    }

    public LimeLight getCameraFacing(double x, double y, boolean relative){
        return getCameraFacing(new Translation2d(x, y), relative);
    }

    public LimeLight getCameraFacing(Translation2d translation2d){
        return getCameraFacing(translation2d, false);
    }

    public LimeLight getCameraFacing(Translation2d translation2d, boolean relative){

        if (vision == null || localization == null) {
            return null;
        }
    
        Pose2d pose = relative ? localization.getRelativePose() : localization.getFieldPose();
        Rotation2d targetAngle = Rotation2d.fromRadians(Math.atan2(translation2d.getY() - pose.getY(), translation2d.getX() - pose.getX()));
        
        Rotation2d desiredRelativeAngle = targetAngle.minus(pose.getRotation());
    
        LimeLight bestCamera = null;
        double smallestAngleDiff = Double.MAX_VALUE;
    
        for (String table : vision.getCameraTables()) {
            LimeLight camera = vision.getCamera(table);
            Rotation2d cameraRelativeAngleRad = camera.config.getRotationRelativeToForwards();
            double angleDiff = Math.abs(desiredRelativeAngle.minus(cameraRelativeAngleRad).getDegrees());
            if (angleDiff < smallestAngleDiff) {
                smallestAngleDiff = angleDiff;
                bestCamera = camera;
            }
        }
        return bestCamera;
    }

    public RotateToPoint rotateTo(double x, double y, boolean relative){
        return new RotateToPoint(this, x, y, relative);
    }

    public RotateToPoint rotateTo(double x, double y){
        return rotateTo(x,y,false);
    }

    public RotateToAngle rotateTo(double degrees){
        return rotateTo(degrees, false);
    }

    public RotateToAngle rotateTo(double degrees, boolean relative){
        return new RotateToAngle(this, degrees, false);
    }

    public RotateToAngle rotateBy(double degrees){
        return new RotateToAngle(this, getDrivetrain().getIMU().getYaw().plus(Rotation2d.fromDegrees(degrees)), false);
    }
}
