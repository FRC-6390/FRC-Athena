package ca.frc6390.athena.core;

import java.util.Map;
import java.util.function.DoubleSupplier;

import ca.frc6390.athena.controllers.EnhancedXboxController;
import ca.frc6390.athena.core.localization.RobotLocalization;
import ca.frc6390.athena.core.localization.RobotLocalizationConfig;
import ca.frc6390.athena.hardware.imu.Imu;
import ca.frc6390.athena.hardware.motor.MotorNeutralMode;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
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
    default ShuffleboardTab shuffleboard(ShuffleboardTab tab, SendableLevel level) {
        
        if(getIMU() != null){
            ShuffleboardLayout imuLayout = tab.getLayout("IMU", BuiltInLayouts.kList).withSize(4, 8);
            getIMU().shuffleboard(imuLayout, level);
        }
       
        if(level.equals(SendableLevel.DEBUG)){
            ShuffleboardLayout speedsLayout = tab.getLayout("Robot Speeds", BuiltInLayouts.kList).withSize(2, 3).withProperties(Map.of("Number of columns", 2, "Number of rows", 1));
            { 
            Map<String, Object> props = Map.of("Min", -getRobotSpeeds().getMaxVelocity(), "Max", getRobotSpeeds().getMaxVelocity(),"Label position", "TOP");
            ShuffleboardLayout chassisLayout = speedsLayout.getLayout("Chassis", BuiltInLayouts.kList).withProperties(Map.of("Number of columns", 1, "Number of rows", 3,"Label position", "TOP"));
            chassisLayout.addDouble("X", () -> getRobotSpeeds().getSpeeds("drive").vxMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
            chassisLayout.addDouble("Y", () -> getRobotSpeeds().getSpeeds("drive").vyMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
            chassisLayout.addDouble("Z", () -> getRobotSpeeds().getSpeeds("drive").omegaRadiansPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
            ShuffleboardLayout feedbackLayout = speedsLayout.getLayout("Feedback", BuiltInLayouts.kList).withProperties(Map.of("Number of columns", 1, "Number of rows", 3,"Label position", "TOP"));
            feedbackLayout.addDouble("X", () -> getRobotSpeeds().getSpeeds("feedback").vxMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
            feedbackLayout.addDouble("Y", () -> getRobotSpeeds().getSpeeds("feedback").vyMetersPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
            feedbackLayout.addDouble("Z", () -> getRobotSpeeds().getSpeeds("feedback").omegaRadiansPerSecond).withWidget(BuiltInWidgets.kNumberBar).withProperties(props);
            }
        }
       

        ShuffleboardLayout commandsLayout = tab.getLayout("Quick Commands",BuiltInLayouts.kList).withSize(1, 3).withProperties(Map.of("Number of columns", 1, "Number of rows", 3));
        {
        commandsLayout.add("Reset Heading", new InstantCommand(() -> getIMU().setYaw(0))).withWidget(BuiltInWidgets.kCommand);
        commandsLayout.add("Brake Mode", new InstantCommand(() -> setNeutralMode(MotorNeutralMode.Brake))).withWidget(BuiltInWidgets.kCommand);
        commandsLayout.add("Coast Mode", new InstantCommand(() -> setNeutralMode(MotorNeutralMode.Coast))).withWidget(BuiltInWidgets.kCommand);
        }

        // ShuffleboardLayout driftCorrectionLayout = tab.getLayout("Drift Correction", BuiltInLayouts.kList).withSize(2, 2);
        // {
        // driftCorrectionLayout.add("Drift Correction", (builder) -> builder.addBooleanProperty("Drift Correction", this::getDriftCorrectionMode, this::setDriftCorrectionMode)).withWidget(BuiltInWidgets.kBooleanBox);
        // driftCorrectionLayout.addDouble("Desired Heading", () -> desiredHeading).withWidget(BuiltInWidgets.kGyro);// might not display properly bc get heading is +-180
        // driftCorrectionLayout.add(driftpid).withWidget(BuiltInWidgets.kPIDController);
        // }
        return tab;
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
