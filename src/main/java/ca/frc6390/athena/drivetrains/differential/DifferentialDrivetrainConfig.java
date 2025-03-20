package ca.frc6390.athena.drivetrains.differential;

import java.util.Arrays;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainConfig;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.DrivetrainIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.DriveIDs;
import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.EncoderIDs;
import ca.frc6390.athena.devices.EncoderConfig;
import ca.frc6390.athena.devices.EncoderGroup;
import ca.frc6390.athena.devices.IMU;
import ca.frc6390.athena.devices.MotorControllerConfig;
import ca.frc6390.athena.devices.MotorControllerGroup;
import ca.frc6390.athena.devices.EncoderConfig.EncoderType;
import ca.frc6390.athena.devices.IMU.IMUConfig;
import ca.frc6390.athena.devices.IMU.IMUType;
import ca.frc6390.athena.devices.MotorController.Motor;

public class DifferentialDrivetrainConfig implements RobotDrivetrainConfig<DifferentialDrivetrain>{
   
    private double maxVelocity;
    private double trackWidth;
    //DEFUALT VALUES
    private IMUConfig imu = null;
    private MotorControllerConfig[] leftMotors, rightMotors;
    private EncoderConfig[] leftEncoders, rightEncoders;
    // private PIDController driftPID = null;
    // private double driftActivationSpeed = 0.05;
    private int[] driveIds = DriveIDs.DUAL_MOTOR_DIFFERENTIAL.getIDs();
    private int[] encoderIds = EncoderIDs.DUAL_MOTOR_DIFFERENTIAL.getIDs();
    private boolean encoderInverted = false;
    private boolean driveInverted = false;
    private double currentLimit = 80;
    private String canbus = "rio";
    private double gearRatio = 1;
    private double wheelDiameterMeters = 1;

    public static DifferentialDrivetrainConfig defualt(IMUType type, boolean inverted, double trackWidth){
        return new DifferentialDrivetrainConfig().setIMU(type, inverted).setTrackWidth(trackWidth);
    }

    public static DifferentialDrivetrainConfig defualt(IMUType type, double trackWidth){
        return new DifferentialDrivetrainConfig().setIMU(type, false).setTrackWidth(trackWidth);
    }

    public DifferentialDrivetrainConfig setIMU(IMUType imu, boolean inverted){
        this.imu = new IMUConfig(imu).setInverted(inverted);
        return this;
    }

    public DifferentialDrivetrainConfig setTrackWidth(double trackWidth){
        this.trackWidth = trackWidth;
        return this;
    }

    public DifferentialDrivetrainConfig setLeftMotors(Motor motor, int... ids){
        return setLeftMotors(Arrays.stream(ids).mapToObj(id -> motor.getMotorControllerType().config(id)).toArray(MotorControllerConfig[]::new));        
    }

    public DifferentialDrivetrainConfig setRightMotors(Motor motor, int... ids){
        return setRightMotors(Arrays.stream(ids).mapToObj(id -> motor.getMotorControllerType().config(id)).toArray(MotorControllerConfig[]::new));        
    }

    public DifferentialDrivetrainConfig setLeftMotors(MotorControllerConfig... config){
        this.leftMotors = config;
        return this;
    }

    public DifferentialDrivetrainConfig setRightMotors(MotorControllerConfig... config){
        this.rightMotors = config;
        return this;
    }

    public DifferentialDrivetrainConfig setLeftEncoders(EncoderType encoder, int... ids){
        return setLeftMotors(Arrays.stream(ids).mapToObj(id -> EncoderConfig.type(encoder, id)).toArray(MotorControllerConfig[]::new));        
    }

    public DifferentialDrivetrainConfig setRightEncoders(EncoderType encoder, int... ids){
        return setRightMotors(Arrays.stream(ids).mapToObj(id -> EncoderConfig.type(encoder, id)).toArray(MotorControllerConfig[]::new));        
    }

    public DifferentialDrivetrainConfig setLeftEncoders(EncoderConfig... config){
        this.leftEncoders = config;
        return this;
    }

    public DifferentialDrivetrainConfig setRightEncoders(EncoderConfig... config){
        this.rightEncoders = config;
        return this;
    }

    public DifferentialDrivetrainConfig setGearRatio(double gearRatio){
        this.gearRatio = gearRatio;
        return this;
    }

    public DifferentialDrivetrainConfig setWheelDiameter(double wheelDiameterMeters){
        this.wheelDiameterMeters = wheelDiameterMeters;
        return this;
    }

    // public DifferentialDrivetrainConfig setDriftPID(double kP, double kI, double kd){
    //     return setDriftPID(new PIDController(kP, kI, kd));
    // }
    
    // public DifferentialDrivetrainConfig setDriftPID(PIDController driftPID){
    //     this.driftPID = driftPID;
    //     return this;
    // }

    // public DifferentialDrivetrainConfig setDriftActivationSpeed(double driftActivationSpeed){
    //     this.driftActivationSpeed = driftActivationSpeed;
    //     return this;
    // }

    public DifferentialDrivetrainConfig setDriveIds(int[] driveIds){
        this.driveIds = driveIds;
        return this;
    } 

    public DifferentialDrivetrainConfig setEncoderIds(int[] encoderIds){
        this.encoderIds = encoderIds;
        return this;
    } 

    public DifferentialDrivetrainConfig setDriveIds(DriveIDs driveIds){
       return setDriveIds(driveIds.getIDs());
    } 

    public DifferentialDrivetrainConfig setEncoderIds(EncoderIDs encoderIds){
        return setEncoderIds(encoderIds.getIDs());
    } 

    public DifferentialDrivetrainConfig setIMUId(int id){
        this.imu = this.imu.setId(id);
        return this;
    }

    public DifferentialDrivetrainConfig setIds(DrivetrainIDs ids){
        setDriveIds(ids.getDrive());
        setEncoderIds(ids.getEncoders());
        setIMUId(ids.getGyro());
        return this;
    } 

    public DifferentialDrivetrainConfig setDriveInverted(boolean driveInverted){
        this.driveInverted = driveInverted;
        return this;
    } 
    
    public DifferentialDrivetrainConfig setEncoderInverted(boolean encoderInverted){
        this.encoderInverted = encoderInverted;
        return this;
    } 

    public DifferentialDrivetrainConfig setCurrentLimit(double currentLimit){
        this.currentLimit = currentLimit;
        return this;
    } 

    public DifferentialDrivetrainConfig setCanbus(String canbus){
        this.canbus = canbus;
        return this;
    } 

    public DifferentialDrivetrainConfig setMaxVelocity(double maxVelocity) {
        this.maxVelocity = maxVelocity;
        return this;
    }

    @Override
    public DifferentialDrivetrain build() {
       
        for (int i = 0; i < leftMotors.length; i++) {
            leftMotors[i] = leftMotors[i].setID(driveIds[i])
                                    .setInverted(driveInverted)
                                    .setCurrentLimit(currentLimit)
                                    .setCanbus(canbus)
                                    .setInverted(driveIds[i] > 0);
        } 

        for (int i = 0; i < rightMotors.length; i++) {
            rightMotors[i] = rightMotors[i].setID(driveIds[i+leftMotors.length])
                                    .setInverted(driveInverted)
                                    .setCurrentLimit(currentLimit)
                                    .setCanbus(canbus)
                                    .setInverted(driveIds[i+leftMotors.length] > 0);
        } 

        if(leftEncoders != null) {
            for (int i = 0; i < leftEncoders.length; i++) {
                leftEncoders[i] = leftEncoders[i].setID(encoderIds[i])
                                        .setInverted(encoderInverted)
                                        .setCanbus(canbus)
                                        .setGearRatio(gearRatio)
                                        .setConversion(Math.PI * wheelDiameterMeters)
                                        .setInverted(encoderIds[i+leftMotors.length] > 0);
            } 
        }

        if(rightEncoders != null) {
            int leftlen = leftEncoders != null ? leftEncoders.length : 0;
            for (int i = 0; i < rightEncoders.length; i++) {
                rightEncoders[i] = rightEncoders[i].setID(encoderIds[i+leftlen])
                                        .setInverted(encoderInverted)
                                        .setCanbus(canbus)
                                        .setGearRatio(gearRatio)
                                        .setConversion(Math.PI *wheelDiameterMeters)
                                        .setInverted(encoderIds[i+leftlen] > 0);
            } 
        }

        if(imu != null){
            imu = imu.setCanbus(canbus);
        }

        MotorControllerGroup lm = new MotorControllerGroup(leftMotors);
        if (leftEncoders != null){
            lm.setEncoders(EncoderGroup.fromConfigs(leftEncoders));
        }

        MotorControllerGroup rm = new MotorControllerGroup(rightMotors);
        if (rightEncoders != null){
            rm.setEncoders(EncoderGroup.fromConfigs(rightEncoders));
        }

        DifferentialDrivetrain dt = new DifferentialDrivetrain(imu == null ? null : IMU.fromConfig(imu), maxVelocity, trackWidth, lm, rm);

        // if (driftPID != null){
        //     dt.setDriftCorrectionPID(driftPID);
        //     dt.setDriftCorrectionMode(true);
        // }

        // dt.setFieldRelative(fieldRelative);
        // dt.driftActivationSpeed = driftActivationSpeed;
        
        return dt;
    }
}


