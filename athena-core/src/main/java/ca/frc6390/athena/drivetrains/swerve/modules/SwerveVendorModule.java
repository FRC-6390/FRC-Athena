package ca.frc6390.athena.drivetrains.swerve.modules;

import ca.frc6390.athena.core.RobotDrivetrain.RobotDrivetrainIDs.DrivetrainIDs;
import ca.frc6390.athena.hardware.encoder.AthenaEncoder;
import ca.frc6390.athena.hardware.motor.AthenaMotor;
import ca.frc6390.athena.hardware.encoder.EncoderConfig;
import ca.frc6390.athena.hardware.encoder.EncoderType;
import ca.frc6390.athena.hardware.motor.MotorControllerConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public interface SwerveVendorModule {

        public double getDriveGearRatio();
        public double getSteerGearRatio();
        public double getEncoderGearRatio();
        public double getWheelDiameter();

        default double getFreeSpeed(AthenaMotor motor) {
            double wheelCircumferenceMeters = Math.PI * getWheelDiameter();
            double motorRPM = motor.getFreeSpeedRPM();
            double wheelRPM = motorRPM * getDriveGearRatio();

            return wheelCircumferenceMeters * (wheelRPM / 60.0);
        }

        default SwerveModuleConfig defualt(){
            return config(null);
        }

        default SwerveModuleConfig config(AthenaMotor drive, AthenaMotor steer){
            return config(null, drive, -1, steer, -1);
        }

        default SwerveModuleConfig config(AthenaMotor motor){
            return config(null, motor, -1, motor, -1);
        }

        default SwerveModuleConfig config(AthenaMotor motor, AthenaEncoder encoder){
            return config(motor, motor, encoder);
        }

        default SwerveModuleConfig config(AthenaMotor drive,  AthenaMotor steer, AthenaEncoder encoder){
            return config(null, drive, -1, steer, -1).setEncoder(EncoderConfig.type(encoder.resolve(), 0).setGearRatio(getEncoderGearRatio()));
        }       

        default SwerveModuleConfig config(AthenaMotor motor, int drive, int steer){
            return config(null, motor, drive, motor, steer);
        }

        default SwerveModuleConfig config(Translation2d location, AthenaMotor motor, int drive, int steer){
            return config(location, motor, drive, motor, steer);
        }

        default SwerveModuleConfig config(Translation2d location, AthenaMotor driveMotor, int drive, AthenaMotor steerMotor, int steer){
            MotorControllerConfig driveMotorController = new MotorControllerConfig(driveMotor.resolveController(), drive)
                    .setEncoderConfig(new EncoderConfig().setConversion(Math.PI * getWheelDiameter()).setGearRatio(getDriveGearRatio()));

            MotorControllerConfig steerMotorController = new MotorControllerConfig(steerMotor.resolveController(), steer)
                    .setEncoderConfig(new EncoderConfig().setGearRatio(getSteerGearRatio()));

            SwerveModuleConfig moduleConfig = new SwerveModuleConfig(location, getWheelDiameter(), getFreeSpeed(driveMotor), driveMotorController, steerMotorController, null, steerMotorController.encoderConfig, SwerveModule.SwerveModuleSimConfig.fromMotors(driveMotor, steerMotor));
            return moduleConfig;
        }

        default SwerveModuleConfig config(Translation2d location, AthenaMotor driveMotor, int drive, AthenaMotor steerMotor, int steer, PIDController rotationPID){
            return config(location, driveMotor, drive, steerMotor, steer).setPID(rotationPID);
        }

        default SwerveModuleConfig config(Translation2d location, AthenaMotor driveMotor, int drive, AthenaMotor steerMotor, int steer, PIDController rotationPID, AthenaEncoder encoderType, int encoder){
            return config(location, driveMotor, drive, steerMotor, steer, rotationPID).setEncoder(EncoderConfig.type(encoderType.resolve(), encoder));
        }

        default SwerveModuleConfig[] configs(Translation2d[] locations, AthenaMotor driveMotor, int[] drive, AthenaMotor steerMotor, int[] steer, PIDController rotationPID, AthenaEncoder encoderType, int[] encoders){
            if (locations.length != drive.length || locations.length != steer.length || locations.length != encoders.length) {
                throw new Error("ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS");
            }
    
            SwerveModuleConfig[] result = new SwerveModuleConfig[locations.length];
        
            for (int i = 0; i < locations.length; i++) {
                result[i] = config(locations[i], driveMotor, drive[i], steerMotor, steer[i], rotationPID, encoderType, encoders[i]);
            }
    
            return result;            
        }

        default SwerveModuleConfig[] configs(DrivetrainIDs IDs, Translation2d[] locations, AthenaMotor driveMotor, AthenaMotor steerMotor, AthenaEncoder encoderType, PIDController rotationPID){
           return configs(locations, driveMotor, IDs.getDrive().getIDs(), steerMotor, IDs.getSteer().getIDs(), rotationPID, encoderType, IDs.getEncoders().getIDs());        
        }

        default SwerveModuleConfig[] configs(DrivetrainIDs IDs, Translation2d[] locations, AthenaMotor motor, AthenaEncoder encoderType, PIDController rotationPID){
            return configs(locations, motor, IDs.getDrive().getIDs(), motor, IDs.getSteer().getIDs(), rotationPID, encoderType, IDs.getEncoders().getIDs());        
         }

        default SwerveModuleConfig[] configs(DrivetrainIDs IDs, double wheelbase, double trackWidth, AthenaMotor motor, AthenaEncoder encoderType, PIDController rotationPID){
            return configs(IDs, wheelbase, trackWidth, motor, motor, encoderType, rotationPID);        
        }

        default SwerveModuleConfig[] configs(DrivetrainIDs IDs, double wheelbase, AthenaMotor motor, AthenaEncoder encoderType, PIDController rotationPID){
            return configs(IDs, wheelbase, wheelbase, motor, motor, encoderType, rotationPID);        
        }

        default SwerveModuleConfig[] configs(DrivetrainIDs IDs, double wheelbase, AthenaMotor driveMotor, AthenaMotor steerMotor, AthenaEncoder encoderType, PIDController rotationPID){
            return configs(IDs, wheelbase, wheelbase, driveMotor, steerMotor, encoderType, rotationPID);        
        }

        default SwerveModuleConfig[] configs(DrivetrainIDs IDs, double wheelbase, double trackWidth, AthenaMotor driveMotor, AthenaMotor steerMotor, AthenaEncoder encoderType, PIDController rotationPID){
            return configs(SwerveModuleConfig.generateModuleLocations(trackWidth, wheelbase), driveMotor, IDs.getDrive().getIDs(), steerMotor, IDs.getSteer().getIDs(), rotationPID, encoderType, IDs.getEncoders().getIDs());        
        }
}
