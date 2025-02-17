package ca.frc6390.athena.drivetrains.tank;

import java.util.function.DoubleSupplier;

import ca.frc6390.athena.commands.control.TankDriveCommand;
import ca.frc6390.athena.core.RobotDrivetrain;
import ca.frc6390.athena.core.RobotLocalization;
import ca.frc6390.athena.core.RobotLocalization.RobotLocalizationConfig;
import ca.frc6390.athena.core.RobotSpeeds;
import ca.frc6390.athena.devices.IMU;
import ca.frc6390.athena.devices.MotorControllerGroup;
import ca.frc6390.athena.devices.IMU.IMUConfig;
import ca.frc6390.athena.devices.MotorController.MotorNeutralMode;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrivetrain extends SubsystemBase implements RobotDrivetrain<TankDrivetrain>{

    public record TankDriveConfig(IMUConfig imu, double maxVelocity) implements RobotDrivetrainConfig<TankDrivetrain> {

        @Override
        public TankDrivetrain create() {
            return new TankDrivetrain(IMU.fromConfig(imu), maxVelocity);
        }
    }

    private final RobotSpeeds speeds;
    private final IMU imu;
    private final DifferentialDrive drive;
    private final MotorControllerGroup leftMotors, rightMotors;

    public TankDrivetrain(IMU imu, double maxVelocity){
        this.imu = imu;
        speeds = new RobotSpeeds(maxVelocity, maxVelocity);

        this.leftMotors = new MotorControllerGroup();
        this.rightMotors = new MotorControllerGroup();
        this.drive = new DifferentialDrive(leftMotors::setSpeed, rightMotors::setSpeed);
    }

    @Override
    public ShuffleboardTab shuffleboard(ShuffleboardTab tab) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'shuffleboard'");
    }

    @Override
    public IMU getIMU() {
        return imu;
    }

    @Override
    public void setNeutralMode(MotorNeutralMode mode) {
       leftMotors.setNeutralMode(mode);
       rightMotors.setNeutralMode(mode);
    }

    @Override
    public RobotSpeeds getRobotSpeeds() {
        return speeds;
    }

    @Override
    public void update() {
        ChassisSpeeds speeds = getRobotSpeeds().calculate();
        drive.arcadeDrive(speeds.vxMetersPerSecond, speeds.omegaRadiansPerSecond);
    }

    @Override
    public Command getDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput){
        return new TankDriveCommand(this, xInput, thetaInput);
    }

    @Override
    public void setDriveCommand(DoubleSupplier xInput, DoubleSupplier yInput, DoubleSupplier thetaInput){
        this.setDefaultCommand(getDriveCommand(xInput, yInput, thetaInput));
    }

    @Override
    public RobotLocalization localization(RobotLocalizationConfig config) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'localization'");
    }
    
    @Override
    public void periodic() {
        update();
    }

    @Override
    public TankDrivetrain get() {
        return this;
    }
}
