package ca.frc6390.athena.core;

import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.core.RobotLocalization.RobotLocalizationConfig;
import ca.frc6390.athena.core.RobotVision.RobotVisionConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain;
import ca.frc6390.athena.drivetrains.swerve.SwerveDrivetrain.SwerveDrivetrainConfig;

public class RobotBase<T extends RobotDrivetrain> {
    
    public record RobotBaseConfig<T extends RobotDrivetrain>(RobotDrivetrainConfig<T> driveTrain, RobotLocalizationConfig localizationConfig, RobotVisionConfig visionConfig) {
        
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

    private final RobotDrivetrain drivetrain;
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

    public RobotDrivetrain getDrivetrain(){
        return drivetrain;
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
}
