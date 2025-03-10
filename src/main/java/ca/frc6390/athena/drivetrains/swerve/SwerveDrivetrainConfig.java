package ca.frc6390.athena.drivetrains.swerve;

import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.DriveIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.EncoderIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.SteerIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.DrivetrainIDs;
import ca.frc6390.athena.devices.IMU;
import ca.frc6390.athena.devices.IMU.IMUConfig;
import ca.frc6390.athena.devices.IMU.IMUType;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveDrivetrainConfig implements RobotDrivetrainConfig<SwerveDrivetrain> {
    
    //DEFUALT VALUES
    private IMUConfig imu = null;
    private SwerveModuleConfig[] modules = null;
    private PIDController driftPID = null;
    private double driftActivationSpeed = 0.05;
    private boolean fieldRelative = true;
    private int[] driveIds = DriveIDs.SWERVE_CHASSIS_STANDARD.getIDs();
    private int[] steerIDs = SteerIDs.SWERVE_CHASSIS_STANDARD.getIDs();
    private int[] encoderIds = EncoderIDs.SWERVE_CHASSIS_STANDARD.getIDs();
    private boolean steerInverted = true;
    private boolean encoderInverted = false;
    private boolean driveInverted = false;
    private PIDController rotationPID = null;
    private double currentLimit = 80;
    private double[] encoderOffsets = {0,0,0,0};
    private String canbus = "rio";
    private Translation2d[] locations = null;

    public static SwerveDrivetrainConfig defualt(Translation2d[] locations){
        return new SwerveDrivetrainConfig().setModuleLocations(locations);
    }

    public static SwerveDrivetrainConfig defualt(double trackWidth){
        return new SwerveDrivetrainConfig().setModuleLocations(trackWidth);
    }

    public static SwerveDrivetrainConfig defualt(double trackWidth, double wheelbase){
        return new SwerveDrivetrainConfig().setModuleLocations(trackWidth, wheelbase);
    }

    public SwerveDrivetrainConfig setIMU(IMUType imu, boolean inverted){
        this.imu = new IMUConfig(imu).setInverted(inverted);
        return this;
    }

    public SwerveDrivetrainConfig modules(SwerveModuleConfig modules){
       return modules(modules, modules, modules, modules);
    }

    public SwerveDrivetrainConfig modules(SwerveModuleConfig... modules){
        this.modules = modules;
        return this;
    }

    public SwerveDrivetrainConfig setDriftPID(double kP, double kI, double kd){
        return setDriftPID(new PIDController(kP, kI, kd));
    }
    
    public SwerveDrivetrainConfig setDriftPID(PIDController driftPID){
        this.driftPID = driftPID;
        return this;
    }

    public SwerveDrivetrainConfig setDriftActivationSpeed(double driftActivationSpeed){
        this.driftActivationSpeed = driftActivationSpeed;
        return this;
    }

    public SwerveDrivetrainConfig setFieldRelative(boolean fieldRelative){
        this.fieldRelative = fieldRelative;
        return this;
    }

    public SwerveDrivetrainConfig setDriveIds(int[] driveIds){
        this.driveIds = driveIds;
        return this;
    } 

    public SwerveDrivetrainConfig setSteerIds(int[] steerIDs){
        this.steerIDs = steerIDs;
        return this;
    } 

    public SwerveDrivetrainConfig setEncoderIds(int[] encoderIds){
        this.encoderIds = encoderIds;
        return this;
    } 

    public SwerveDrivetrainConfig setDriveIds(DriveIDs driveIds){
       return setDriveIds(driveIds.getIDs());
    } 

    public SwerveDrivetrainConfig setSteerIds(SteerIDs steerIDs){
        return setSteerIds(steerIDs.getIDs());
    } 

    public SwerveDrivetrainConfig setEncoderIds(EncoderIDs encoderIds){
        return setEncoderIds(encoderIds.getIDs());
    } 

    public SwerveDrivetrainConfig setIMUId(int id){
        this.imu = this.imu.setId(id);
        return this;
    }

    public SwerveDrivetrainConfig setIds(DrivetrainIDs ids){
        setDriveIds(ids.getDrive());
        setSteerIds(ids.getSteer());
        setEncoderIds(ids.getEncoders());
        setIMUId(ids.getGyro());
        return this;
    } 

    public SwerveDrivetrainConfig setDriveInverted(boolean driveInverted){
        this.driveInverted = driveInverted;
        return this;
    } 

    public SwerveDrivetrainConfig setSteerInverted(boolean steerInverted){
        this.steerInverted = steerInverted;
        return this;
    } 

    public SwerveDrivetrainConfig setEncoderInverted(boolean encoderInverted){
        this.encoderInverted = encoderInverted;
        return this;
    } 

    public SwerveDrivetrainConfig setRotationPID(double kP, double kI, double kd){
        return setRotationPID(new PIDController(kP, kI, kd));
    }
    
    public SwerveDrivetrainConfig setRotationPID(PIDController rotationPID){
        this.rotationPID = rotationPID;
        return this;
    }
    
    public SwerveDrivetrainConfig setCurrentLimit(double currentLimit){
        this.currentLimit = currentLimit;
        return this;
    } 

    public SwerveDrivetrainConfig setEncoderOffset(double... encoderOffsets){
        this.encoderOffsets = encoderOffsets;
        return this;
    } 

    public SwerveDrivetrainConfig setCanbus(String canbus){
        this.canbus = canbus;
        return this;
    } 

    public SwerveDrivetrainConfig setModuleLocations(Translation2d[] locations){
        this.locations = locations;
        return this;
    }

    public SwerveDrivetrainConfig setModuleLocations(double trackWidth, double wheelbase){
        return setModuleLocations(SwerveModuleConfig.generateModuleLocations(trackWidth, wheelbase));
    }

    public SwerveDrivetrainConfig setModuleLocations(double trackWidth){
        return setModuleLocations(SwerveModuleConfig.generateModuleLocations(trackWidth, trackWidth));
    }


    @Override
    public SwerveDrivetrain build() {
        if (encoderIds.length != modules.length) {
        throw new Error("ENCODER ID ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS (expected: "+modules.length +", got: "+encoderIds.length+")");
        }

        if (driveIds.length != modules.length) {
        throw new Error("DRIVE ID ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS (expected: "+modules.length +", got: "+driveIds.length+")");
        }

        if (steerIDs.length != modules.length) {
        throw new Error("STEER ID ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS (expected: "+modules.length +", got: "+steerIDs.length+")");
        }

        if (encoderOffsets.length != modules.length) {
        throw new Error("ENCODER OFFSETS ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS (expected: "+modules.length +", got: "+encoderOffsets.length+")");
        }

        if (locations.length != modules.length) {
        throw new Error("MODULE LOCATIONS ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS (expected: "+modules.length +", got: "+locations.length+")");
        }

        for (int i = 0; i < modules.length; i++) {
            modules[i] = modules[i].setEncoderID(encoderIds[i])
                                    .setSteerID(steerIDs[i])
                                    .setDriveID(driveIds[i])
                                    .setEncoderInverted(encoderInverted)
                                    .setSteerInverted(steerInverted)
                                    .setDriveInverted(driveInverted)
                                    .setCurrentLimit(currentLimit)
                                    .setOffset(encoderOffsets[i])
                                    .setCanbus(canbus)
                                    .setLocation(locations[i]);
            
            if(rotationPID != null){
                modules[i] = modules[i].setPID(rotationPID);
            }
        } 

        imu = imu.setCanbus(canbus);

        SwerveDrivetrain dt = new SwerveDrivetrain(IMU.fromConfig(imu), modules);

        if (driftPID != null){
            dt.setDriftCorrectionPID(driftPID);
            dt.setDriftCorrectionMode(true);
        }

        dt.setFieldRelative(fieldRelative);
        dt.driftActivationSpeed = driftActivationSpeed;
        return dt;
    }
}