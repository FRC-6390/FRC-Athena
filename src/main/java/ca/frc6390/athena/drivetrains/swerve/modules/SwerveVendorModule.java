package ca.frc6390.athena.drivetrains.swerve.modules;

import ca.frc6390.athena.core.RobotDrivetrain.RobotDriveTrainIDs.DrivetrainIDs;
import ca.frc6390.athena.devices.Encoder;
import ca.frc6390.athena.devices.MotorController;
import ca.frc6390.athena.devices.Encoder.EncoderConfig;
import ca.frc6390.athena.devices.Encoder.EncoderType;
import ca.frc6390.athena.devices.MotorController.MotorControllerConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public interface SwerveVendorModule {

        public double getDriveGearRatio();
        public double getSteerGearRatio();
        public double getEncoderGearRatio();
        public double getWheelDiameter();

        default double getFreeSpeed(MotorController.Motor motor) {
            double wheelCircumferenceMeters = Math.PI * getWheelDiameter();
            double motorRPM = motor.getFreeSpeedRPM();
            double wheelRPM = motorRPM * getDriveGearRatio();

            return wheelCircumferenceMeters * (wheelRPM / 60.0);
        }

        default SwerveModuleConfig config(MotorController.Motor drive, MotorController.Motor steer){
            return config(null, drive, -1, steer, -1);
        }

        default SwerveModuleConfig config(MotorController.Motor motor){
            return config(null, motor, -1, motor, -1);
        }

        default SwerveModuleConfig config(MotorController.Motor motor, EncoderType type){
            return config( motor, motor, type);
        }

        default SwerveModuleConfig config(MotorController.Motor drive,  MotorController.Motor steer, EncoderType type){
            return config(null, drive, -1, steer, -1).setEncoder(new EncoderConfig(type).setGearRatio(getEncoderGearRatio()));
        }       

        default SwerveModuleConfig config(MotorController.Motor motor, int drive, int steer){
            return config(null, motor, drive, motor, steer);
        }

        default SwerveModuleConfig config(Translation2d location, MotorController.Motor motor, int drive, int steer){
            return config(location, motor, drive, motor, steer);
        }

        default SwerveModuleConfig config(Translation2d location, MotorController.Motor driveMotor, int drive, MotorController.Motor steerMotor, int steer){
    
            MotorControllerConfig driveMotorController = new MotorControllerConfig(driveMotor.getMotorControllerType(), drive);
            driveMotorController.withEncoderConfig( new EncoderConfig().setConversion(Math.PI * getWheelDiameter()).setGearRatio(getDriveGearRatio()));

            MotorControllerConfig steerMotorController = new MotorControllerConfig(steerMotor.getMotorControllerType(), steer);
            steerMotorController.withEncoderConfig(new EncoderConfig().setGearRatio(getSteerGearRatio()));

            SwerveModuleConfig moduleConfig = new SwerveModuleConfig(location, getWheelDiameter(), getFreeSpeed(driveMotor), driveMotorController, steerMotorController, null);
           
            return moduleConfig;
        }

        default SwerveModuleConfig config(Translation2d location, MotorController.Motor driveMotor, int drive, MotorController.Motor steerMotor, int steer, PIDController rotationPID){
            return config(location, driveMotor, drive, steerMotor, steer).setPID(rotationPID);
        }

        default SwerveModuleConfig config(Translation2d location, MotorController.Motor driveMotor, int drive, MotorController.Motor steerMotor, int steer, PIDController rotationPID, Encoder.EncoderType encoderType, int encoder){
            return config(location, driveMotor, drive, steerMotor, steer, rotationPID).setEncoder(new EncoderConfig(encoderType, encoder));
        }

        default SwerveModuleConfig[] configs(Translation2d[] locations, MotorController.Motor driveMotor, int[] drive, MotorController.Motor steerMotor, int[] steer, PIDController rotationPID, Encoder.EncoderType encoderType, int[] encoders){
            if (locations.length != drive.length || locations.length != steer.length || locations.length != encoders.length) {
                throw new Error("ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS");
            }
    
            SwerveModuleConfig[] result = new SwerveModuleConfig[locations.length];
        
            for (int i = 0; i < locations.length; i++) {
                result[i] = config(locations[i], driveMotor, drive[i], steerMotor, steer[i], rotationPID, encoderType, encoders[i]);
            }
    
            return result;            
        }

        default SwerveModuleConfig[] configs(DrivetrainIDs IDs, Translation2d[] locations, MotorController.Motor driveMotor, MotorController.Motor steerMotor, Encoder.EncoderType encoderType, PIDController rotationPID){
           return configs(locations, driveMotor, IDs.getDrive().getIDs(), steerMotor, IDs.getSteer().getIDs(), rotationPID, encoderType, IDs.getEncoders().getIDs());        
        }

        default SwerveModuleConfig[] configs(DrivetrainIDs IDs, Translation2d[] locations, MotorController.Motor motor, Encoder.EncoderType encoderType, PIDController rotationPID){
            return configs(locations, motor, IDs.getDrive().getIDs(), motor, IDs.getSteer().getIDs(), rotationPID, encoderType, IDs.getEncoders().getIDs());        
         }

        default SwerveModuleConfig[] configs(DrivetrainIDs IDs, double wheelbase, double trackWidth, MotorController.Motor motor, Encoder.EncoderType encoderType, PIDController rotationPID){
            return configs(IDs, wheelbase, trackWidth, motor, motor, encoderType, rotationPID);        
        }

        default SwerveModuleConfig[] configs(DrivetrainIDs IDs, double wheelbase, MotorController.Motor motor, Encoder.EncoderType encoderType, PIDController rotationPID){
            return configs(IDs, wheelbase, wheelbase, motor, motor, encoderType, rotationPID);        
        }

        default SwerveModuleConfig[] configs(DrivetrainIDs IDs, double wheelbase, MotorController.Motor driveMotor, MotorController.Motor steerMotor, Encoder.EncoderType encoderType, PIDController rotationPID){
            return configs(IDs, wheelbase, wheelbase, driveMotor, steerMotor, encoderType, rotationPID);        
        }

        default SwerveModuleConfig[] configs(DrivetrainIDs IDs, double wheelbase, double trackWidth, MotorController.Motor driveMotor, MotorController.Motor steerMotor, Encoder.EncoderType encoderType, PIDController rotationPID){
            return configs(SwerveModuleConfig.generateModuleLocations(trackWidth, wheelbase), driveMotor, IDs.getDrive().getIDs(), steerMotor, IDs.getSteer().getIDs(), rotationPID, encoderType, IDs.getEncoders().getIDs());        
        }
}
