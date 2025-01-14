package ca.frc6390.athena.drivetrains.swerve;

import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveEncoder;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveModuleConfig;
import ca.frc6390.athena.drivetrains.swerve.SwerveModule.SwerveMotor;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveHelpers {
 
    public static Translation2d[] generateModuleLocations(double trackwidth, double wheelbase) {
        Translation2d FRONT_LEFT = new Translation2d(trackwidth/2, wheelbase/2);
        Translation2d FRONT_RIGHT = new Translation2d(trackwidth/2, -wheelbase/2);
        Translation2d BACK_LEFT = new Translation2d(-trackwidth/2, wheelbase/2);
        Translation2d BACK_RIGHT = new Translation2d(-trackwidth/2, -wheelbase/2);

        return new Translation2d[]{FRONT_LEFT, FRONT_RIGHT, BACK_LEFT, BACK_RIGHT};
    }


    public static SwerveMotor[] generateMotors(double gearRatio, double maxSpeedMetersPerSecond, String canbus ,int... ids) {

        SwerveMotor[] result = new SwerveMotor[ids.length];
    
        for (int i = 0; i < ids.length; i++) {
            result[i] = new SwerveMotor(ids[i], maxSpeedMetersPerSecond, gearRatio, canbus);
        }

        return result;
    }

    public static SwerveMotor[] generateMotors(double gearRatio, double maxSpeedMetersPerSecond ,int... ids) {

        SwerveMotor[] result = new SwerveMotor[ids.length];
    
        for (int i = 0; i < ids.length; i++) {
            result[i] = new SwerveMotor(ids[i], maxSpeedMetersPerSecond, gearRatio);
        }

        return result;
    }

    public static SwerveEncoder[] generateEncoders(double gearRatio, double[] offsetRotations, String canbus, int... ids) {

        if (ids.length != offsetRotations.length) {
            throw new Error("ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE ENCODERS");
        }

        SwerveEncoder[] result = new SwerveEncoder[ids.length];
    
        for (int i = 0; i < ids.length; i++) {
            result[i] = new SwerveEncoder(ids[i], offsetRotations[i], gearRatio, canbus);
        }

        return result;
    }

    public static SwerveEncoder[] generateEncoders(double gearRatio, double[] offsetRotations, int... ids) {

        if (ids.length != offsetRotations.length) {
            throw new Error("ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE ENCODERS");
        }

        SwerveEncoder[] result = new SwerveEncoder[ids.length];
    
        for (int i = 0; i < ids.length; i++) {
            result[i] = new SwerveEncoder(ids[i], offsetRotations[i], gearRatio);
        }

        return result;
    }

    public static SwerveModuleConfig[] generateConfigs(Translation2d[] locations, double wheelDiameter, SwerveMotor[] drive, SwerveMotor[] rotation, PIDController rotationPID, SwerveEncoder[] encoders) {
        
        if (locations.length != drive.length || locations.length != rotation.length || locations.length != encoders.length) {
            throw new Error("ARRAY LENGTHS DO NOT MATCH TO GENERATE SWERVEMODULE CONFIGS");
        }

        SwerveModuleConfig[] result = new SwerveModuleConfig[locations.length];
    
        for (int i = 0; i < locations.length; i++) {
            result[i] = new SwerveModuleConfig(locations[i], wheelDiameter, drive[i], rotation[i], rotationPID, encoders[i]);
        }

        return result;
    }
}
