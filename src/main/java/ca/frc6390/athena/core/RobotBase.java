package ca.frc6390.athena.core;

import ca.frc6390.athena.commands.movement.RotateToAngle;
import ca.frc6390.athena.commands.movement.RotateToPoint;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.core.RobotLocalization.RobotLocalizationConfig;
import ca.frc6390.athena.core.RobotVision.RobotVisionConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain.SwerveDrivetrainConfig;
import edu.wpi.first.math.geometry.Rotation2d;

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

    public RotateToPoint rotateTo(double x, double y, boolean relative){
        return new RotateToPoint(this, x, y, relative);
    }

    public RotateToPoint rotateTo(double x, double y){
        return rotateTo(x,y,true);
    }

    public RotateToAngle rotateTo(double degrees){
        return rotateTo(degrees, true);
    }

    public RotateToAngle rotateTo(double degrees, boolean relative){
        return new RotateToAngle(this, degrees, false);
    }

    public RotateToAngle rotateBy(double degrees){
        return new RotateToAngle(this, getDrivetrain().getIMU().getYaw().plus(Rotation2d.fromDegrees(degrees)), true);
    }
}
