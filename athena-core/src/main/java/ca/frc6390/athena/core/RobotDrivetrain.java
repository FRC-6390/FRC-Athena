package ca.frc6390.athena.core;

import java.util.Map;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import ca.frc6390.athena.core.RobotNetworkTables;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface RobotDrivetrain<T extends RobotDrivetrain<T>> extends RobotSendableSystem, Subsystem {
    
    public interface RobotDrivetrainConfig<T extends RobotDrivetrain<T>> {
        T build();
    }

    T get();
    Imu getIMU();
    void setNeutralMode(MotorNeutralMode mode);
    MotionLimits getMotionLimits();
    RobotSpeeds getRobotSpeeds();
    void update();
    Command getDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput);
    void setDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput);
    RobotLocalization<?> localization(RobotLocalizationConfig config);

    default void setDriveCommand(EnhancedXboxController driverController) {
        setDriveCommand(driverController.leftX, driverController.leftY, driverController.rightX);
    }

    @Override
    default RobotNetworkTables.Node networkTables(RobotNetworkTables.Node node) {
        if (node == null) {
            return node;
        }
        RobotNetworkTables nt = node.robot();
        if (!nt.isPublishingEnabled()) {
            return node;
        }

        if (getIMU() != null) {
            getIMU().networkTables(node.child("IMU"));
        }

        if (nt.enabled(RobotNetworkTables.Flag.DRIVETRAIN_SPEED_WIDGETS)) {
            RobotNetworkTables.Node speeds = node.child("RobotSpeeds");
            speeds.child("drive").putDouble("vxMps", getRobotSpeeds().getSpeeds("drive").vxMetersPerSecond);
            speeds.child("drive").putDouble("vyMps", getRobotSpeeds().getSpeeds("drive").vyMetersPerSecond);
            speeds.child("drive").putDouble("omegaRadPerSec", getRobotSpeeds().getSpeeds("drive").omegaRadiansPerSecond);

            speeds.child("feedback").putDouble("vxMps", getRobotSpeeds().getSpeeds("feedback").vxMetersPerSecond);
            speeds.child("feedback").putDouble("vyMps", getRobotSpeeds().getSpeeds("feedback").vyMetersPerSecond);
            speeds.child("feedback").putDouble("omegaRadPerSec", getRobotSpeeds().getSpeeds("feedback").omegaRadiansPerSecond);
        }

        // Commands are intentionally not published here; dashboards can invoke actions via your own bindings.
        return node;
    }

    public class RobotDrivetrainIDs {

        public enum DrivetrainIDs {
            SWERVE_CHASSIS_STANDARD(DriveIDs.SWERVE_CHASSIS_STANDARD, SteerIDs.SWERVE_CHASSIS_STANDARD, EncoderIDs.SWERVE_CHASSIS_STANDARD, 0),
            DUAL_MOTOR_DIFFERENTIAL(DriveIDs.DUAL_MOTOR_DIFFERENTIAL, SteerIDs.DUAL_MOTOR_DIFFERENTIAL, EncoderIDs.DUAL_MOTOR_DIFFERENTIAL, 0);

            private final DriveIDs drive;
            private final SteerIDs steer;
            private final EncoderIDs encoders;
            private final int gyro;
    
            DrivetrainIDs(DriveIDs drive, SteerIDs steer, EncoderIDs encoders, int gyro) {
                this.drive = drive;
                this.steer = steer;
                this.encoders = encoders;
                this.gyro = gyro;
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

            public int getGyro(){
                return gyro;
            }
        }
    
        public enum DriveIDs {
            SWERVE_CHASSIS_STANDARD(new int[] {1,2,3,4}),
            DUAL_MOTOR_DIFFERENTIAL(new int[] {1,2,3,4});

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
            DUAL_MOTOR_DIFFERENTIAL(new int[] {});

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
            DUAL_MOTOR_DIFFERENTIAL(new int[] {5,6});
    
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
