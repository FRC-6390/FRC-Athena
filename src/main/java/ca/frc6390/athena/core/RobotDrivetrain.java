package ca.frc6390.athena.core;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.devices.IMU;
import ca.frc6390.athena.devices.MotorController.MotorNeutralMode;
import edu.wpi.first.wpilibj2.command.Command;

public interface RobotDrivetrain {
    
    public record RobotDrivetrainConfig() {
        
    }

    IMU getIMU();
    void setNeutralMode(MotorNeutralMode mode);
    RobotSpeeds getRobotSpeeds();
    void update();
    Command getDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput);
    void setDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput);


    public class RobotDriveTrainIDs {

        public enum DrivetrainIDs {
            SWERVE_CHASSIS_STANDARD(DriveIDs.SWERVE_CHASSIS_STANDARD, SteerIDs.SWERVE_CHASSIS_STANDARD, EncoderIDs.SWERVE_CHASSIS_STANDARD),
            DUAL_MOTOR_TANK(DriveIDs.DUAL_MOTOR_TANK, SteerIDs.DUAL_MOTOR_TANK, EncoderIDs.DUAL_MOTOR_TANK);

            private final DriveIDs drive;
            private final SteerIDs steer;
            private final EncoderIDs encoders;
    
            DrivetrainIDs(DriveIDs drive, SteerIDs steer, EncoderIDs encoders) {
                this.drive = drive;
                this.steer = steer;
                this.encoders = encoders;
            }
    
            public DriveIDs getDrive() {
                return drive;
            }

            public EncoderIDs getEncoders() {
                return encoders;
            }

            public SteerIDs getSteer() {
                return steer;
            }
        }
    
        public enum DriveIDs {
            SWERVE_CHASSIS_STANDARD(new int[] {1,2,3,4}),
            DUAL_MOTOR_TANK(new int[] {1,2,3,4});

            private final int[] ids;
    
            DriveIDs(int[] ids) {
                this.ids = ids;
            }
    
            public int[] getIDs(){
                return ids;
            }
        }
    
        public enum SteerIDs {
            SWERVE_CHASSIS_STANDARD(new int[] {5,6,7,8}),
            DUAL_MOTOR_TANK(new int[] {});

            private final int[] ids;
    
            SteerIDs(int[] ids) {
                this.ids = ids;
            }
    
            public int[] getIDs(){
                return ids;
            }
        }
    
        public enum EncoderIDs {
            SWERVE_CHASSIS_STANDARD(new int[] {9,10,11,12}),
            DUAL_MOTOR_TANK(new int[] {5,6});
    
            private final int[] ids;
    
            EncoderIDs(int[] ids) {
                this.ids = ids;
            }
    
            public int[] getIDs(){
                return ids;
            }
        }
    }
}
